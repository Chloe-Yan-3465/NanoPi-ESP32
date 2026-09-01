#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <limits>
#include <sched.h>
#include <signal.h>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

#include "../common/camera_control_shm.h"

static std::atomic<bool> g_running{true};
static constexpr int CAMERA_RESPONSE_TIMEOUT_MS = 2500;
static constexpr size_t UART_LINE_MAX = 256;

static void signal_handler(int)
{
    g_running.store(false);
}

static long long ts_to_ns(const timespec& ts)
{
    return static_cast<long long>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

static timespec ns_to_ts(long long ns)
{
    timespec ts{};
    ts.tv_sec = ns / 1000000000LL;
    ts.tv_nsec = ns % 1000000000LL;
    return ts;
}

static long long clock_now_ns(clockid_t clk)
{
    timespec ts{};
    clock_gettime(clk, &ts);
    return ts_to_ns(ts);
}

static speed_t baud_to_flag(int baud)
{
    switch (baud)
    {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 500000: return B500000;
        case 576000: return B576000;
        case 921600: return B921600;
        case 1000000: return B1000000;
        case 1500000: return B1500000;
        default:
            std::cerr << "[ERR] Unsupported baud: " << baud << std::endl;
            std::exit(1);
    }
}

static int open_serial(const char* dev, int baud)
{
    int fd = open(dev, O_RDWR | O_NOCTTY | O_CLOEXEC);
    if (fd < 0)
    {
        std::cerr << "[ERR] open serial failed: " << dev
                  << ", " << strerror(errno) << std::endl;
        std::exit(1);
    }

    termios tio{};
    if (tcgetattr(fd, &tio) != 0)
    {
        std::cerr << "[ERR] tcgetattr failed: "
                  << strerror(errno) << std::endl;
        close(fd);
        std::exit(1);
    }

    cfmakeraw(&tio);
    speed_t speed = baud_to_flag(baud);
    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);

    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    tio.c_cflag &= ~CRTSCTS;
    tio.c_cflag |= CREAD | CLOCAL;

    // Wake at least every 100 ms so camera responses are returned well within
    // the ESP32's three-second ACK deadline.
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 1;

    tcflush(fd, TCIOFLUSH);
    if (tcsetattr(fd, TCSANOW, &tio) != 0)
    {
        std::cerr << "[ERR] tcsetattr failed: "
                  << strerror(errno) << std::endl;
        close(fd);
        std::exit(1);
    }
    return fd;
}

static std::string trim_line(std::string value)
{
    while (!value.empty() &&
           (value.back() == '\n' || value.back() == '\r' ||
            value.back() == ' ' || value.back() == '\t'))
    {
        value.pop_back();
    }

    size_t start = 0;
    while (start < value.size() &&
           (value[start] == ' ' || value[start] == '\t'))
    {
        ++start;
    }
    if (start > 0)
        value.erase(0, start);
    return value;
}

static bool parse_timesync_line(const std::string& raw_line,
                                long long& unix_ns)
{
    const std::string line = trim_line(raw_line);
    const std::string prefix = "TIMESYNC+";
    const std::string suffix = "+END";

    if (line.size() <= prefix.size() + suffix.size() ||
        line.rfind(prefix, 0) != 0 ||
        line.compare(line.size() - suffix.size(), suffix.size(), suffix) != 0)
    {
        return false;
    }

    std::string utc = line.substr(
        prefix.size(), line.size() - prefix.size() - suffix.size());
    if (utc.size() != 27 ||
        utc[4] != '-' || utc[7] != '-' || utc[10] != 'T' ||
        utc[13] != ':' || utc[16] != ':' || utc[19] != '.' ||
        utc[26] != 'Z')
    {
        return false;
    }

    for (size_t i = 0; i < utc.size(); ++i)
    {
        if (i == 4 || i == 7 || i == 10 || i == 13 ||
            i == 16 || i == 19 || i == 26)
        {
            continue;
        }
        if (!std::isdigit(static_cast<unsigned char>(utc[i])))
            return false;
    }

    int year = std::atoi(utc.substr(0, 4).c_str());
    int mon = std::atoi(utc.substr(5, 2).c_str());
    int mday = std::atoi(utc.substr(8, 2).c_str());
    int hour = std::atoi(utc.substr(11, 2).c_str());
    int min = std::atoi(utc.substr(14, 2).c_str());
    int sec = std::atoi(utc.substr(17, 2).c_str());
    int usec = std::atoi(utc.substr(20, 6).c_str());

    if (year < 1970 || mon < 1 || mon > 12 ||
        mday < 1 || mday > 31 || hour < 0 || hour > 23 ||
        min < 0 || min > 59 || sec < 0 || sec > 60 ||
        usec < 0 || usec > 999999)
    {
        return false;
    }

    tm tm_utc{};
    tm_utc.tm_year = year - 1900;
    tm_utc.tm_mon = mon - 1;
    tm_utc.tm_mday = mday;
    tm_utc.tm_hour = hour;
    tm_utc.tm_min = min;
    tm_utc.tm_sec = sec;
    tm_utc.tm_isdst = 0;

    errno = 0;
    time_t unix_sec = timegm(&tm_utc);
    if (unix_sec == static_cast<time_t>(-1) && errno != 0)
        return false;

    tm check{};
    if (gmtime_r(&unix_sec, &check) == nullptr ||
        check.tm_year != tm_utc.tm_year ||
        check.tm_mon != tm_utc.tm_mon ||
        check.tm_mday != tm_utc.tm_mday ||
        check.tm_hour != tm_utc.tm_hour ||
        check.tm_min != tm_utc.tm_min ||
        check.tm_sec != tm_utc.tm_sec)
    {
        return false;
    }

    unix_ns = static_cast<long long>(unix_sec) * 1000000000LL
            + static_cast<long long>(usec) * 1000LL;
    return unix_ns > 0;
}

static bool set_realtime_ns(long long target_realtime_ns)
{
    timespec ts = ns_to_ts(target_realtime_ns);
    if (clock_settime(CLOCK_REALTIME, &ts) != 0)
    {
        std::cerr << "[ERR] clock_settime failed: "
                  << strerror(errno) << std::endl;
        return false;
    }
    return true;
}

static void touch_synced_file()
{
    std::ofstream file("/tmp/uart_timesync_synced");
    if (file.is_open())
        file << "synced\n";
}

static void try_realtime_priority()
{
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
    {
        std::cerr << "[WARN] mlockall failed: "
                  << strerror(errno) << std::endl;
    }

    sched_param sp{};
    sp.sched_priority = 80;
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0)
    {
        std::cerr << "[WARN] sched_setscheduler failed: "
                  << strerror(errno)
                  << " ; continue with normal scheduler" << std::endl;
    }
}

struct SharedMemoryConnection
{
    int fd = -1;
    CameraControlSharedMemory* memory = nullptr;
};

static void disconnect_shared_memory(SharedMemoryConnection& connection)
{
    if (connection.memory != nullptr)
    {
        munmap(connection.memory, sizeof(CameraControlSharedMemory));
        connection.memory = nullptr;
    }
    if (connection.fd >= 0)
    {
        close(connection.fd);
        connection.fd = -1;
    }
}

static bool connect_shared_memory(SharedMemoryConnection& connection)
{
    if (connection.memory != nullptr)
        return true;

    int fd = shm_open(CAMERA_CONTROL_SHM_NAME, O_RDWR, 0666);
    if (fd < 0)
        return false;

    struct stat info{};
    if (fstat(fd, &info) != 0 ||
        info.st_size < static_cast<off_t>(sizeof(CameraControlSharedMemory)))
    {
        close(fd);
        return false;
    }

    void* mapped = mmap(nullptr, sizeof(CameraControlSharedMemory),
                        PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (mapped == MAP_FAILED)
    {
        close(fd);
        return false;
    }

    auto* memory = static_cast<CameraControlSharedMemory*>(mapped);
    if (memory->magic != CAMERA_CONTROL_SHM_MAGIC ||
        memory->version != CAMERA_CONTROL_SHM_VERSION)
    {
        munmap(mapped, sizeof(CameraControlSharedMemory));
        close(fd);
        return false;
    }

    connection.fd = fd;
    connection.memory = memory;
    std::cout << "[OK] Camera shared memory connected" << std::endl;
    return true;
}

enum class ParsedCommandType
{
    NONE,
    START,
    STOP,
    STATUS
};

struct ParsedCommand
{
    ParsedCommandType type = ParsedCommandType::NONE;
    std::uint32_t session = 0;
    std::string task_name = "test";
    std::string complex_level = "L_test";
};

static bool parse_uint32_exact(const std::string& text, std::uint32_t& value)
{
    if (text.empty())
        return false;
    for (char ch : text)
    {
        if (!std::isdigit(static_cast<unsigned char>(ch)))
            return false;
    }

    errno = 0;
    char* end = nullptr;
    unsigned long long parsed = std::strtoull(text.c_str(), &end, 10);
    if (errno != 0 || end == nullptr || *end != '\0' ||
        parsed > std::numeric_limits<std::uint32_t>::max())
    {
        return false;
    }
    value = static_cast<std::uint32_t>(parsed);
    return true;
}

static bool parse_command_line(const std::string& raw_line,
                               ParsedCommand& command)
{
    const std::string line = trim_line(raw_line);
    const std::string suffix = "+END";
    if (line.size() <= suffix.size() ||
        line.compare(line.size() - suffix.size(), suffix.size(), suffix) != 0)
    {
        return false;
    }

    struct Prefix
    {
        const char* text;
        ParsedCommandType type;
    };
    static const Prefix prefixes[] = {
        {"CMD+START+SESSION=", ParsedCommandType::START},
        {"CMD+STOP+SESSION=", ParsedCommandType::STOP},
        {"CMD+STATUS+SESSION=", ParsedCommandType::STATUS}
    };

    for (const Prefix& prefix : prefixes)
    {
        std::string prefix_text(prefix.text);
        if (line.rfind(prefix_text, 0) != 0)
            continue;

        const std::string payload = line.substr(
            prefix_text.size(),
            line.size() - prefix_text.size() - suffix.size());
        const size_t session_end = payload.find('+');
        const std::string session_text = payload.substr(0, session_end);
        std::uint32_t session = 0;
        if (!parse_uint32_exact(session_text, session))
            return false;

        std::string task_name = "test";
        std::string complex_level = "L_test";
        if (prefix.type == ParsedCommandType::START &&
            session_end != std::string::npos)
        {
            const std::string metadata = payload.substr(session_end);
            const std::string task_prefix = "+TASK=";
            const std::string level_marker = "+LEVEL=";
            if (metadata.rfind(task_prefix, 0) != 0)
                return false;
            const size_t level_start = metadata.find(level_marker,
                                                      task_prefix.size());
            if (level_start == std::string::npos)
                return false;
            task_name = metadata.substr(
                task_prefix.size(), level_start - task_prefix.size());
            complex_level = metadata.substr(level_start + level_marker.size());
            const auto valid_segment = [](const std::string& value,
                                          size_t capacity)
            {
                if (value.empty() || value.size() >= capacity)
                    return false;
                return std::all_of(value.begin(), value.end(), [](char ch)
                {
                    const unsigned char byte = static_cast<unsigned char>(ch);
                    return std::isalnum(byte) || ch == '_' || ch == '-';
                });
            };
            if (!valid_segment(task_name, CAMERA_TASK_NAME_CAPACITY) ||
                !valid_segment(complex_level, CAMERA_COMPLEX_LEVEL_CAPACITY))
            {
                return false;
            }
        }
        else if (session_end != std::string::npos)
        {
            return false;
        }

        command.type = prefix.type;
        command.session = session;
        command.task_name = task_name;
        command.complex_level = complex_level;
        return true;
    }
    return false;
}

static const char* command_name(ParsedCommandType type)
{
    switch (type)
    {
        case ParsedCommandType::START: return "START";
        case ParsedCommandType::STOP: return "STOP";
        case ParsedCommandType::STATUS: return "STATUS";
        default: return "UNKNOWN";
    }
}

static CameraControlCommand to_shared_command(ParsedCommandType type)
{
    switch (type)
    {
        case ParsedCommandType::START: return CAMERA_COMMAND_START;
        case ParsedCommandType::STOP: return CAMERA_COMMAND_STOP;
        case ParsedCommandType::STATUS: return CAMERA_COMMAND_STATUS;
        default: return CAMERA_COMMAND_NONE;
    }
}

static const char* state_name(std::uint32_t state)
{
    switch (state)
    {
        case CAMERA_STATE_IDLE: return "IDLE";
        case CAMERA_STATE_STARTING: return "STARTING";
        case CAMERA_STATE_RUNNING: return "RUNNING";
        case CAMERA_STATE_STOPPING: return "STOPPING";
        case CAMERA_STATE_ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

static std::string sanitize_error(const char* raw)
{
    std::string value = raw != nullptr ? raw : "";
    if (value.empty())
        value = "CAMERA_ERROR";
    for (char& ch : value)
    {
        unsigned char byte = static_cast<unsigned char>(ch);
        if (!(std::isalnum(byte) || ch == '_'))
            ch = '_';
    }
    return value;
}

static bool write_all(int fd, const std::string& value)
{
    size_t offset = 0;
    while (offset < value.size())
    {
        ssize_t count = write(fd, value.data() + offset,
                              value.size() - offset);
        if (count < 0)
        {
            if (errno == EINTR)
                continue;
            std::cerr << "[ERR] UART write failed: "
                      << strerror(errno) << std::endl;
            return false;
        }
        offset += static_cast<size_t>(count);
    }
    return true;
}

static void send_line(int fd, const std::string& frame)
{
    const std::string wire = frame + "\r\n";
    if (write_all(fd, wire))
        std::cout << "[UART TX] " << frame << std::endl;
}

static void send_immediate_error(int fd, const ParsedCommand& command,
                                 const std::string& error)
{
    if (command.type == ParsedCommandType::STATUS)
    {
        send_line(fd, "STATUS+SESSION=" + std::to_string(command.session) +
                      "+ERROR+" + error + "+END");
        return;
    }

    send_line(fd, "ACK+" + std::string(command_name(command.type)) +
                  "+SESSION=" + std::to_string(command.session) +
                  "+ERROR+" + error + "+END");
}

struct PendingCommand
{
    bool active = false;
    ParsedCommand command;
    std::uint32_t request_seq = 0;
    std::chrono::steady_clock::time_point deadline;
};

static void submit_camera_command(int serial_fd,
                                  SharedMemoryConnection& shared,
                                  PendingCommand& pending,
                                  const ParsedCommand& command)
{
    if (pending.active)
    {
        send_immediate_error(serial_fd, command, "RECEIVER_BUSY");
        return;
    }
    if (!connect_shared_memory(shared))
    {
        send_immediate_error(serial_fd, command, "CAMERA_NOT_READY");
        return;
    }

    std::uint32_t sequence =
        camera_shm_load_u32(&shared.memory->request_seq) + 1U;
    if (sequence == 0)
        sequence = 1;

    camera_shm_store_u32(&shared.memory->command,
                         to_shared_command(command.type));
    camera_shm_store_u32(&shared.memory->session_id, command.session);
    std::memset(shared.memory->task_name, 0,
                sizeof(shared.memory->task_name));
    std::memset(shared.memory->complex_level, 0,
                sizeof(shared.memory->complex_level));
    std::snprintf(shared.memory->task_name,
                  sizeof(shared.memory->task_name), "%s",
                  command.task_name.c_str());
    std::snprintf(shared.memory->complex_level,
                  sizeof(shared.memory->complex_level), "%s",
                  command.complex_level.c_str());
    camera_shm_store_u32(&shared.memory->request_seq, sequence);

    pending.active = true;
    pending.command = command;
    pending.request_seq = sequence;
    pending.deadline = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(CAMERA_RESPONSE_TIMEOUT_MS);

    std::cout << "[CMD] Forwarded " << command_name(command.type)
              << " session=" << command.session
              << " request_seq=" << sequence << std::endl;
}

static void poll_camera_response(int serial_fd,
                                 SharedMemoryConnection& shared,
                                 PendingCommand& pending)
{
    if (!pending.active)
        return;

    if (shared.memory == nullptr)
    {
        send_immediate_error(serial_fd, pending.command,
                             "CAMERA_NOT_READY");
        pending.active = false;
        return;
    }

    std::uint32_t response_seq =
        camera_shm_load_u32(&shared.memory->response_seq);
    if (response_seq == pending.request_seq)
    {
        std::int32_t result =
            camera_shm_load_i32(&shared.memory->result_code);
        std::uint32_t state =
            camera_shm_load_u32(&shared.memory->state);
        std::uint64_t frames =
            camera_shm_load_u64(&shared.memory->frames_written);
        std::string error = sanitize_error(shared.memory->error);

        if (result == CAMERA_RESULT_OK)
        {
            if (pending.command.type == ParsedCommandType::START)
            {
                send_line(serial_fd,
                    "ACK+START+SESSION=" +
                    std::to_string(pending.command.session) + "+OK+END");
            }
            else if (pending.command.type == ParsedCommandType::STOP)
            {
                send_line(serial_fd,
                    "ACK+STOP+SESSION=" +
                    std::to_string(pending.command.session) +
                    "+OK+FRAMES=" + std::to_string(frames) + "+END");
            }
            else
            {
                send_line(serial_fd,
                    "STATUS+SESSION=" +
                    std::to_string(pending.command.session) + "+" +
                    state_name(state) + "+FRAMES=" +
                    std::to_string(frames) + "+END");
            }
        }
        else
        {
            send_immediate_error(serial_fd, pending.command, error);
        }
        pending.active = false;
        return;
    }

    if (std::chrono::steady_clock::now() >= pending.deadline)
    {
        send_immediate_error(serial_fd, pending.command,
                             "CAMERA_RESPONSE_TIMEOUT");
        pending.active = false;
    }
}

static bool camera_recording_active(const SharedMemoryConnection& shared)
{
    if (shared.memory == nullptr)
        return false;

    const std::uint32_t state =
        camera_shm_load_u32(&shared.memory->state);
    return state == CAMERA_STATE_STARTING ||
           state == CAMERA_STATE_RUNNING ||
           state == CAMERA_STATE_STOPPING;
}

static bool should_set_time(const std::string& set_mode,
                            bool already_set_once,
                            bool recording_active)
{
    return set_mode == "every" ||
           (set_mode == "idle" && !recording_active) ||
           (set_mode == "once" && !already_set_once);
}

static void usage(const char* program)
{
    std::cerr
        << "Usage:\n  sudo " << program
        << " <serial_dev> <baud> [set_mode]\n\n"
        << "Example:\n  sudo " << program
        << " /dev/ttyS1 115200 idle\n\n"
        << "set_mode: idle | every | once | none\n"
        << "  idle: set CLOCK_REALTIME only while camera is not recording (default)\n\n"
        << "Accepted UART frames:\n"
        << "  TIMESYNC+YYYY-MM-DDTHH:MM:SS.ffffffZ+END\\r\\n\n"
        << "  CMD+START+SESSION=<id>+TASK=<name>+LEVEL=<level>+END\\r\\n\n"
        << "  CMD+STOP+SESSION=<id>+END\\r\\n\n"
        << "  CMD+STATUS+SESSION=<id>+END\\r\\n\n";
}

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        usage(argv[0]);
        return 1;
    }

    const char* serial_dev = argv[1];
    int baud = std::atoi(argv[2]);
    std::string set_mode = argc >= 4 ? argv[3] : "idle";
    if (set_mode != "idle" && set_mode != "every" &&
        set_mode != "once" && set_mode != "none")
    {
        std::cerr << "[ERR] invalid set_mode: " << set_mode << std::endl;
        usage(argv[0]);
        return 1;
    }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    std::cout << "[INFO] UART time-sync and camera receiver starting\n"
              << "[INFO] serial=" << serial_dev << ", baud=" << baud
              << ", set_mode=" << set_mode << std::endl;

    try_realtime_priority();
    int serial_fd = open_serial(serial_dev, baud);

    SharedMemoryConnection shared;
    if (!connect_shared_memory(shared))
    {
        std::cerr << "[WARN] Camera shared memory is not ready; "
                  << "commands will return CAMERA_NOT_READY" << std::endl;
    }

    PendingCommand pending;
    std::string line;
    bool already_set_once = false;
    bool none_mode_synced_file_touched = false;
    unsigned long long timesync_seq = 0;

    while (g_running.load())
    {
        char buffer[128];
        ssize_t count = read(serial_fd, buffer, sizeof(buffer));
        if (count < 0)
        {
            if (errno != EINTR)
            {
                std::cerr << "[ERR] serial read failed: "
                          << strerror(errno) << std::endl;
                usleep(100000);
            }
        }
        else
        {
            for (ssize_t i = 0; i < count; ++i)
            {
                char ch = buffer[i];
                if (ch == '\n')
                {
                    std::string frame = trim_line(line);
                    line.clear();
                    if (frame.empty())
                        continue;

                    long long uart_unix_ns = 0;
                    if (parse_timesync_line(frame, uart_unix_ns))
                    {
                        ++timesync_seq;
                        long long before_ns = clock_now_ns(CLOCK_REALTIME);
                        std::cout << "[TIMESYNC] seq=" << timesync_seq
                                  << " target_ns=" << uart_unix_ns
                                  << " diff_before_ms="
                                  << (uart_unix_ns - before_ns) / 1000000.0
                                  << std::endl;

                        const bool recording_active =
                            camera_recording_active(shared);
                        if (should_set_time(set_mode, already_set_once,
                                            recording_active))
                        {
                            if (set_realtime_ns(uart_unix_ns))
                            {
                                already_set_once = true;
                                touch_synced_file();
                                long long after_ns =
                                    clock_now_ns(CLOCK_REALTIME);
                                std::cout << "[OK] CLOCK_REALTIME updated; "
                                          << "error_after_set_ms="
                                          << (after_ns - uart_unix_ns) /
                                             1000000.0
                                          << std::endl;
                            }
                        }
                        else
                        {
                            if (set_mode == "none" &&
                                !none_mode_synced_file_touched)
                            {
                                touch_synced_file();
                                none_mode_synced_file_touched = true;
                            }
                            std::cout << "[INFO] system time unchanged due to "
                                      << "set_mode=" << set_mode;
                            if (set_mode == "idle" && recording_active)
                                std::cout << " camera_recording=1";
                            std::cout << std::endl;
                        }
                        continue;
                    }

                    ParsedCommand command;
                    if (parse_command_line(frame, command))
                    {
                        submit_camera_command(serial_fd, shared,
                                              pending, command);
                    }
                    else
                    {
                        std::cerr << "[WARN] invalid UART line: "
                                  << frame << std::endl;
                    }
                }
                else if (line.size() < UART_LINE_MAX)
                {
                    line.push_back(ch);
                }
                else
                {
                    std::cerr << "[WARN] UART line too long, dropped"
                              << std::endl;
                    line.clear();
                }
            }
        }

        poll_camera_response(serial_fd, shared, pending);
    }

    disconnect_shared_memory(shared);
    close(serial_fd);
    return 0;
}
