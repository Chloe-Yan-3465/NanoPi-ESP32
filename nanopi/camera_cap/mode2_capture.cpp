#include <librealsense2/rs.hpp>
#include <yaml-cpp/yaml.h>

#define MODULE_TAG "mode2_capture"
extern "C"
{
#include <rockchip/rk_mpi.h>
}

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

#include <fcntl.h>
#include <sched.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include "camera_control_shm.h"

namespace fs = std::filesystem;

namespace
{

volatile sig_atomic_t g_running = 1;

void signal_handler(int)
{
    g_running = 0;
}

std::int64_t unix_now_ns()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

std::string local_timestamp()
{
    const auto now = std::chrono::system_clock::now();
    const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm tm_value{};
    localtime_r(&now_time, &tm_value);

    const auto milliseconds = std::chrono::duration_cast<
        std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000;
    char date_time[32]{};
    std::strftime(date_time, sizeof(date_time), "%Y%m%d_%H%M%S", &tm_value);

    char result[40]{};
    std::snprintf(result, sizeof(result), "%s_%03lld", date_time,
                  static_cast<long long>(milliseconds));
    return result;
}

void bind_current_thread(int core, const char* thread_name)
{
    if (core < 0)
        return;

    long cpu_count = sysconf(_SC_NPROCESSORS_ONLN);
    if (core >= cpu_count)
    {
        throw std::runtime_error(
            std::string("CPU core out of range for ") + thread_name +
            ": " + std::to_string(core));
    }

    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(core, &set);
    if (sched_setaffinity(0, sizeof(set), &set) != 0)
    {
        throw std::runtime_error(
            std::string("sched_setaffinity failed for ") + thread_name +
            ": " + std::strerror(errno));
    }
    std::cout << "[CPU] " << thread_name << " -> core " << core << '\n';
}

template <typename T>
T yaml_value(const YAML::Node& parent, const char* key, const T& fallback)
{
    const YAML::Node value = parent[key];
    return value ? value.as<T>() : fallback;
}

struct AppConfig
{
    std::string config_path;
    std::string serial;
    std::string camera_name = "camera";
    int width = 1280;
    int height = 720;
    int fps = 30;
    std::size_t max_mjpeg_bytes = 2U * 1024U * 1024U;
    int jpeg_quality = 80;
    int sync_mode = 2;

    bool disable_auto_exposure = true;
    float color_exposure = 150.0F;
    float depth_exposure = 8000.0F;
    float color_gain = 16.0F;
    float depth_gain = 16.0F;
    bool disable_auto_exposure_priority = true;
    bool global_time_enabled = false;

    std::string base_dir = "/home/pi/data_mode2";
    bool write_depth = true;
    std::size_t queue_capacity = 24;
    std::uint32_t chunk_frames = 300;
    bool fdatasync_on_chunk_close = true;
    int drain_timeout_ms = 15000;

    int capture_core = 1;
    int encoder_core = -1;
    int writer_core = -1;
    std::string control_mode = "shared_memory";
};

AppConfig load_config(const std::string& path)
{
    YAML::Node root = YAML::LoadFile(path);
    AppConfig cfg;
    cfg.config_path = path;

    const YAML::Node device = root["device"];
    const YAML::Node streams = root["streams"];
    const YAML::Node sync = root["sync"];
    const YAML::Node controls = root["camera_controls"];
    const YAML::Node storage = root["storage"];
    const YAML::Node affinity = root["cpu_affinity"];
    const YAML::Node control = root["control"];

    cfg.serial = yaml_value<std::string>(device, "serial", "");
    cfg.camera_name = yaml_value<std::string>(device, "camera_name", "camera");
    cfg.width = yaml_value<int>(streams, "width", 1280);
    cfg.height = yaml_value<int>(streams, "height", 720);
    cfg.fps = yaml_value<int>(streams, "fps", 30);
    cfg.max_mjpeg_bytes = yaml_value<std::size_t>(
        streams, "max_mjpeg_bytes", 2U * 1024U * 1024U);
    cfg.jpeg_quality = yaml_value<int>(streams, "jpeg_quality", 80);

    const std::string color_format =
        yaml_value<std::string>(streams, "color_format", "yuyv");
    const std::string depth_format =
        yaml_value<std::string>(streams, "depth_format", "z16");
    if (color_format != "yuyv" || depth_format != "z16")
    {
        throw std::runtime_error(
            "This build requires streams.color_format=yuyv and "
            "streams.depth_format=z16");
    }

    cfg.sync_mode = yaml_value<int>(sync, "inter_cam_sync_mode", 2);
    cfg.disable_auto_exposure = yaml_value<bool>(
        controls, "disable_auto_exposure", true);
    cfg.color_exposure = yaml_value<float>(
        controls, "color_exposure", 150.0F);
    cfg.depth_exposure = yaml_value<float>(
        controls, "depth_exposure", 8000.0F);
    cfg.color_gain = yaml_value<float>(controls, "color_gain", 16.0F);
    cfg.depth_gain = yaml_value<float>(controls, "depth_gain", 16.0F);
    cfg.disable_auto_exposure_priority = yaml_value<bool>(
        controls, "disable_auto_exposure_priority", true);
    cfg.global_time_enabled = yaml_value<bool>(
        controls, "global_time_enabled", false);

    cfg.base_dir = yaml_value<std::string>(
        storage, "base_dir", "/home/pi/data_mode2");
    cfg.write_depth = yaml_value<bool>(storage, "write_depth", true);
    cfg.queue_capacity = yaml_value<std::size_t>(
        storage, "queue_capacity", 24);
    cfg.chunk_frames = yaml_value<std::uint32_t>(
        storage, "chunk_frames", 300);
    cfg.fdatasync_on_chunk_close = yaml_value<bool>(
        storage, "fdatasync_on_chunk_close", true);
    cfg.drain_timeout_ms = yaml_value<int>(
        storage, "drain_timeout_ms", 15000);

    cfg.capture_core = yaml_value<int>(affinity, "capture_core", 1);
    cfg.encoder_core = yaml_value<int>(affinity, "encoder_core", -1);
    cfg.writer_core = yaml_value<int>(affinity, "writer_core", -1);
    cfg.control_mode = yaml_value<std::string>(
        control, "mode", "shared_memory");

    if (cfg.width <= 0 || cfg.height <= 0 || cfg.fps <= 0 ||
        (cfg.width & 1) != 0)
        throw std::runtime_error("Invalid stream dimensions or FPS");
    if (cfg.camera_name.empty() || cfg.camera_name.size() > 64U ||
        cfg.camera_name.find_first_not_of(
            "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_-") !=
            std::string::npos)
    {
        throw std::runtime_error(
            "device.camera_name must contain only A-Z, a-z, 0-9, '_' or '-'");
    }
    if (cfg.sync_mode != 0 && cfg.sync_mode != 2)
        throw std::runtime_error(
            "inter_cam_sync_mode must be 0 (free-run) or 2 (external slave)");
    if (cfg.queue_capacity < 2 || cfg.queue_capacity > 120)
        throw std::runtime_error("queue_capacity must be in [2, 120]");
    if (cfg.chunk_frames == 0)
        throw std::runtime_error("chunk_frames must be > 0");
    const std::size_t yuyv_bytes = static_cast<std::size_t>(cfg.width) *
        static_cast<std::size_t>(cfg.height) * 2U;
    if (cfg.max_mjpeg_bytes < yuyv_bytes)
    {
        throw std::runtime_error(
            "max_mjpeg_bytes must also fit one packed YUYV input frame");
    }
    if (cfg.jpeg_quality < 1 || cfg.jpeg_quality > 99)
        throw std::runtime_error("jpeg_quality must be in [1, 99]");
    if (cfg.control_mode != "shared_memory" &&
        cfg.control_mode != "immediate")
    {
        throw std::runtime_error(
            "control.mode must be shared_memory or immediate");
    }
    return cfg;
}

std::int64_t metadata_or_minus_one(
    const rs2::frame& frame, rs2_frame_metadata_value key)
{
    try
    {
        if (frame.supports_frame_metadata(key))
        {
            return static_cast<std::int64_t>(frame.get_frame_metadata(key));
        }
    }
    catch (...)
    {
    }
    return -1;
}

struct FrameBundle
{
    explicit FrameBundle(std::size_t rgb_capacity, std::size_t depth_capacity)
        : rgb(rgb_capacity), depth(depth_capacity)
    {
    }

    std::vector<std::uint8_t> rgb;
    std::vector<std::uint8_t> depth;
    std::uint32_t rgb_bytes = 0;
    std::uint32_t depth_bytes = 0;
    std::uint64_t sequence = 0;
    std::uint64_t rgb_frame_number = 0;
    std::uint64_t depth_frame_number = 0;
    std::int64_t host_receive_unix_ns = 0;
    double rgb_rs_timestamp_ms = 0.0;
    double depth_rs_timestamp_ms = 0.0;
    std::int64_t rgb_sensor_timestamp = -1;
    std::int64_t depth_sensor_timestamp = -1;
    std::uint32_t rgb_timestamp_domain = 0;
    std::uint32_t depth_timestamp_domain = 0;
};

class FramePool
{
public:
    FramePool(std::size_t count, std::size_t rgb_capacity,
              std::size_t depth_capacity)
    {
        storage_.reserve(count);
        for (std::size_t i = 0; i < count; ++i)
        {
            storage_.push_back(std::make_unique<FrameBundle>(
                rgb_capacity, depth_capacity));
            free_.push_back(storage_.back().get());
        }
    }

    FrameBundle* try_acquire()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (free_.empty())
            return nullptr;
        FrameBundle* item = free_.front();
        free_.pop_front();
        ++capture_active_;
        return item;
    }

    void submit_for_encode(FrameBundle* item)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (capture_active_ == 0)
                throw std::logic_error("FramePool capture_active underflow");
            encode_ready_.push_back(item);
            --capture_active_;
            update_peaks_locked();
        }
        encode_cv_.notify_one();
    }

    bool pop_for_encode(FrameBundle*& item, int timeout_ms)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!encode_cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms),
                                 [this] { return !encode_ready_.empty(); }))
        {
            return false;
        }
        item = encode_ready_.front();
        encode_ready_.pop_front();
        ++encode_active_;
        return true;
    }

    void submit_for_write(FrameBundle* item)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (encode_active_ == 0)
                throw std::logic_error("FramePool encode_active underflow");
            write_ready_.push_back(item);
            --encode_active_;
            update_peaks_locked();
        }
        write_cv_.notify_one();
    }

    bool pop_for_write(FrameBundle*& item, int timeout_ms)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!write_cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms),
                                [this] { return !write_ready_.empty(); }))
        {
            return false;
        }
        item = write_ready_.front();
        write_ready_.pop_front();
        ++write_active_;
        return true;
    }

    void release(FrameBundle* item)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (capture_active_ == 0)
            throw std::logic_error("FramePool capture_active underflow");
        --capture_active_;
        free_.push_back(item);
    }

    void release_after_encode(FrameBundle* item)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (encode_active_ == 0)
            throw std::logic_error("FramePool encode_active underflow");
        --encode_active_;
        free_.push_back(item);
    }

    void release_after_write(FrameBundle* item)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (write_active_ == 0)
            throw std::logic_error("FramePool write_active underflow");
        --write_active_;
        free_.push_back(item);
    }

    void clear_queued()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        while (!encode_ready_.empty())
        {
            free_.push_back(encode_ready_.front());
            encode_ready_.pop_front();
        }
        while (!write_ready_.empty())
        {
            free_.push_back(write_ready_.front());
            write_ready_.pop_front();
        }
    }

    std::size_t encode_size() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return encode_ready_.size();
    }

    std::size_t write_size() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return write_ready_.size();
    }

    std::size_t queued_size() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return encode_ready_.size() + write_ready_.size();
    }

    bool idle() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return encode_ready_.empty() && write_ready_.empty() &&
            capture_active_ == 0 && encode_active_ == 0 &&
            write_active_ == 0;
    }

    std::size_t peak_encode() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return peak_encode_;
    }

    std::size_t peak_write() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return peak_write_;
    }

    std::size_t peak_total() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return peak_total_;
    }

private:
    void update_peaks_locked()
    {
        peak_encode_ = std::max(peak_encode_, encode_ready_.size());
        peak_write_ = std::max(peak_write_, write_ready_.size());
        peak_total_ = std::max(
            peak_total_, encode_ready_.size() + write_ready_.size());
    }

    std::vector<std::unique_ptr<FrameBundle>> storage_;
    std::deque<FrameBundle*> free_;
    std::deque<FrameBundle*> encode_ready_;
    std::deque<FrameBundle*> write_ready_;
    mutable std::mutex mutex_;
    std::condition_variable encode_cv_;
    std::condition_variable write_cv_;
    std::size_t peak_encode_ = 0;
    std::size_t peak_write_ = 0;
    std::size_t peak_total_ = 0;
    std::size_t capture_active_ = 0;
    std::size_t encode_active_ = 0;
    std::size_t write_active_ = 0;
};

#pragma pack(push, 1)
struct IndexHeader
{
    char magic[8];                 // RSDIDX3\0
    std::uint32_t version;
    std::uint32_t header_bytes;
    std::uint32_t record_bytes;
    std::uint32_t width;
    std::uint32_t height;
    std::uint32_t fps;
    std::uint32_t chunk_id;
    std::uint32_t session_id;
    std::uint32_t rgb_fourcc;      // MJPG
    std::uint32_t depth_fourcc;    // Z16 + space
    float depth_scale;
    std::int64_t created_unix_ns;
    std::uint64_t record_count;    // finalized with pwrite on clean close
    std::uint8_t reserved[32];
};

struct IndexRecord
{
    std::uint64_t sequence;
    std::uint64_t rgb_frame_number;
    std::uint64_t depth_frame_number;
    std::int64_t host_receive_unix_ns;
    double rgb_rs_timestamp_ms;
    double depth_rs_timestamp_ms;
    std::int64_t rgb_sensor_timestamp;
    std::int64_t depth_sensor_timestamp;
    std::uint32_t rgb_timestamp_domain;
    std::uint32_t depth_timestamp_domain;
    std::uint64_t rgb_offset;
    std::uint64_t depth_offset;
    std::uint32_t rgb_bytes;
    std::uint32_t depth_bytes;
    std::uint32_t flags;
    std::uint32_t reserved;
};
#pragma pack(pop)

static_assert(std::is_trivially_copyable<IndexHeader>::value,
              "IndexHeader must be binary writable");
static_assert(std::is_trivially_copyable<IndexRecord>::value,
              "IndexRecord must be binary writable");

constexpr std::uint32_t fourcc(char a, char b, char c, char d)
{
    return static_cast<std::uint32_t>(static_cast<unsigned char>(a)) |
           (static_cast<std::uint32_t>(static_cast<unsigned char>(b)) << 8U) |
           (static_cast<std::uint32_t>(static_cast<unsigned char>(c)) << 16U) |
           (static_cast<std::uint32_t>(static_cast<unsigned char>(d)) << 24U);
}

void write_all(int fd, const void* data, std::size_t bytes)
{
    const auto* cursor = static_cast<const std::uint8_t*>(data);
    std::size_t remaining = bytes;
    while (remaining > 0)
    {
        ssize_t count = ::write(fd, cursor, remaining);
        if (count < 0)
        {
            if (errno == EINTR)
                continue;
            throw std::runtime_error(
                std::string("write failed: ") + std::strerror(errno));
        }
        if (count == 0)
            throw std::runtime_error("write returned zero");
        cursor += count;
        remaining -= static_cast<std::size_t>(count);
    }
}

void pwrite_all(int fd, const void* data, std::size_t bytes, off_t offset)
{
    const auto* cursor = static_cast<const std::uint8_t*>(data);
    std::size_t remaining = bytes;
    while (remaining > 0)
    {
        ssize_t count = ::pwrite(fd, cursor, remaining, offset);
        if (count < 0)
        {
            if (errno == EINTR)
                continue;
            throw std::runtime_error(
                std::string("pwrite failed: ") + std::strerror(errno));
        }
        if (count == 0)
            throw std::runtime_error("pwrite returned zero");
        cursor += count;
        remaining -= static_cast<std::size_t>(count);
        offset += count;
    }
}

int open_new_file(const fs::path& path)
{
    int fd = ::open(path.c_str(), O_CREAT | O_EXCL | O_WRONLY | O_CLOEXEC,
                    0644);
    if (fd < 0)
    {
        throw std::runtime_error(
            "open failed for " + path.string() + ": " + std::strerror(errno));
    }
    return fd;
}

class ChunkFiles
{
public:
    ~ChunkFiles()
    {
        close_noexcept();
    }

    void open(const fs::path& directory, std::uint32_t chunk_id,
              std::uint32_t session_id, const AppConfig& cfg,
              float depth_scale)
    {
        close();
        chunk_id_ = chunk_id;
        write_depth_ = cfg.write_depth;
        char id[16]{};
        std::snprintf(id, sizeof(id), "%06u", chunk_id);

        rgb_fd_ = open_new_file(directory / (std::string("rgb_") + id + ".mjpg"));
        try
        {
            if (write_depth_)
            {
                depth_fd_ = open_new_file(
                    directory / (std::string("depth_") + id + ".z16"));
            }
            index_fd_ = open_new_file(
                directory / (std::string("index_") + id + ".bin"));
        }
        catch (...)
        {
            close_noexcept();
            throw;
        }

        IndexHeader header{};
        std::memcpy(header.magic, "RSDIDX3", 7);
        header.version = 1;
        header.header_bytes = sizeof(IndexHeader);
        header.record_bytes = sizeof(IndexRecord);
        header.width = static_cast<std::uint32_t>(cfg.width);
        header.height = static_cast<std::uint32_t>(cfg.height);
        header.fps = static_cast<std::uint32_t>(cfg.fps);
        header.chunk_id = chunk_id;
        header.session_id = session_id;
        header.rgb_fourcc = fourcc('M', 'J', 'P', 'G');
        header.depth_fourcc = write_depth_
            ? fourcc('Z', '1', '6', ' ')
            : fourcc('N', 'O', 'N', 'E');
        header.depth_scale = depth_scale;
        header.created_unix_ns = unix_now_ns();
        write_all(index_fd_, &header, sizeof(header));
    }

    void append(const FrameBundle& frame)
    {
        if (!is_open())
            throw std::runtime_error("append called with no open chunk");

        IndexRecord record{};
        record.sequence = frame.sequence;
        record.rgb_frame_number = frame.rgb_frame_number;
        record.depth_frame_number = frame.depth_frame_number;
        record.host_receive_unix_ns = frame.host_receive_unix_ns;
        record.rgb_rs_timestamp_ms = frame.rgb_rs_timestamp_ms;
        record.depth_rs_timestamp_ms = frame.depth_rs_timestamp_ms;
        record.rgb_sensor_timestamp = frame.rgb_sensor_timestamp;
        record.depth_sensor_timestamp = frame.depth_sensor_timestamp;
        record.rgb_timestamp_domain = frame.rgb_timestamp_domain;
        record.depth_timestamp_domain = frame.depth_timestamp_domain;
        record.rgb_offset = rgb_offset_;
        record.depth_offset = depth_offset_;
        record.rgb_bytes = frame.rgb_bytes;
        record.depth_bytes = write_depth_ ? frame.depth_bytes : 0U;
        if (frame.rgb_sensor_timestamp >= 0)
            record.flags |= 1U;
        if (frame.depth_sensor_timestamp >= 0)
            record.flags |= 2U;

        // Data first, index last: a crash cannot leave an index record that
        // points beyond data that was never submitted to the kernel.
        write_all(rgb_fd_, frame.rgb.data(), frame.rgb_bytes);
        if (write_depth_)
            write_all(depth_fd_, frame.depth.data(), frame.depth_bytes);
        write_all(index_fd_, &record, sizeof(record));

        rgb_offset_ += frame.rgb_bytes;
        if (write_depth_)
            depth_offset_ += frame.depth_bytes;
        ++record_count_;
    }

    void close(bool fdatasync_enabled)
    {
        if (!is_open())
            return;
        const off_t count_offset = static_cast<off_t>(
            offsetof(IndexHeader, record_count));
        pwrite_all(index_fd_, &record_count_, sizeof(record_count_), count_offset);

        if (fdatasync_enabled)
        {
            if (::fdatasync(rgb_fd_) != 0 ||
                (write_depth_ && ::fdatasync(depth_fd_) != 0) ||
                ::fdatasync(index_fd_) != 0)
            {
                throw std::runtime_error(
                    std::string("fdatasync failed: ") + std::strerror(errno));
            }
        }
        close_descriptors();
    }

    void close()
    {
        if (is_open())
            close(false);
    }

    bool is_open() const { return index_fd_ >= 0; }
    std::uint64_t record_count() const { return record_count_; }

private:
    void close_descriptors()
    {
        if (rgb_fd_ >= 0) ::close(rgb_fd_);
        if (depth_fd_ >= 0) ::close(depth_fd_);
        if (index_fd_ >= 0) ::close(index_fd_);
        rgb_fd_ = depth_fd_ = index_fd_ = -1;
        rgb_offset_ = depth_offset_ = record_count_ = 0;
        write_depth_ = true;
    }

    void close_noexcept()
    {
        try
        {
            close_descriptors();
        }
        catch (...)
        {
        }
    }

    int rgb_fd_ = -1;
    int depth_fd_ = -1;
    int index_fd_ = -1;
    std::uint32_t chunk_id_ = 0;
    std::uint64_t rgb_offset_ = 0;
    std::uint64_t depth_offset_ = 0;
    std::uint64_t record_count_ = 0;
    bool write_depth_ = true;
};

} // namespace

namespace
{

class SharedControl
{
public:
    SharedControl()
    {
        shm_unlink(CAMERA_CONTROL_SHM_NAME);
        int fd = shm_open(CAMERA_CONTROL_SHM_NAME,
                          O_CREAT | O_EXCL | O_RDWR, 0666);
        if (fd < 0)
        {
            throw std::runtime_error(
                std::string("shm_open failed: ") + std::strerror(errno));
        }
        if (ftruncate(fd, sizeof(CameraControlSharedMemory)) != 0)
        {
            const std::string error = std::strerror(errno);
            ::close(fd);
            shm_unlink(CAMERA_CONTROL_SHM_NAME);
            throw std::runtime_error("ftruncate failed: " + error);
        }

        void* mapped = mmap(nullptr, sizeof(CameraControlSharedMemory),
                            PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        ::close(fd);
        if (mapped == MAP_FAILED)
        {
            shm_unlink(CAMERA_CONTROL_SHM_NAME);
            throw std::runtime_error(
                std::string("mmap failed: ") + std::strerror(errno));
        }

        memory_ = static_cast<CameraControlSharedMemory*>(mapped);
        std::memset(memory_, 0, sizeof(*memory_));
        memory_->magic = CAMERA_CONTROL_SHM_MAGIC;
        memory_->version = CAMERA_CONTROL_SHM_VERSION;
        camera_shm_store_u32(&memory_->state, CAMERA_STATE_STARTING);
        camera_shm_store_i32(&memory_->result_code, CAMERA_RESULT_OK);
    }

    ~SharedControl()
    {
        if (memory_ != nullptr)
            munmap(memory_, sizeof(CameraControlSharedMemory));
        shm_unlink(CAMERA_CONTROL_SHM_NAME);
    }

    CameraControlSharedMemory* get() { return memory_; }

    void set_state(CameraControlState state)
    {
        camera_shm_store_u32(&memory_->state, state);
    }

    void set_frames(std::uint64_t frames)
    {
        camera_shm_store_u64(&memory_->frames_written, frames);
    }

    void set_ep_path(const std::string& path)
    {
        std::memset(memory_->ep_path, 0, sizeof(memory_->ep_path));
        std::snprintf(memory_->ep_path, sizeof(memory_->ep_path), "%s",
                      path.c_str());
    }

    void respond(std::uint32_t request_seq, CameraControlResult result,
                 const std::string& error)
    {
        camera_shm_store_i32(&memory_->result_code, result);
        std::memset(memory_->error, 0, sizeof(memory_->error));
        if (!error.empty())
        {
            std::snprintf(memory_->error, sizeof(memory_->error), "%s",
                          error.c_str());
        }
        camera_shm_store_u32(&memory_->response_seq, request_seq);
    }

private:
    CameraControlSharedMemory* memory_ = nullptr;
};

void install_signal_handlers()
{
    struct sigaction action{};
    action.sa_handler = signal_handler;
    sigemptyset(&action.sa_mask);
    action.sa_flags = 0;
    if (sigaction(SIGINT, &action, nullptr) != 0 ||
        sigaction(SIGTERM, &action, nullptr) != 0)
    {
        throw std::runtime_error("sigaction failed");
    }
}

void usage(const char* program)
{
    std::cerr << "Usage: " << program << " [config.yaml]\n";
}

} // namespace

namespace
{
rs2::device select_device(rs2::context& context, const std::string& serial);
}

template <typename WriterT, typename EncoderT, typename CaptureT>
int run_main(int argc, char** argv)
{
    if (argc > 2)
    {
        usage(argv[0]);
        return 2;
    }

    try
    {
        install_signal_handlers();
        const std::string config_path = argc == 2
            ? argv[1]
            : "/home/pi/camera_cap/config.yaml";
        AppConfig cfg = load_config(config_path);

        const std::size_t depth_bytes =
            static_cast<std::size_t>(cfg.width) *
            static_cast<std::size_t>(cfg.height) * sizeof(std::uint16_t);
        const double pool_mib =
            static_cast<double>(cfg.queue_capacity) *
            static_cast<double>(cfg.max_mjpeg_bytes + depth_bytes) /
            1024.0 / 1024.0;

        std::cout << "[CONFIG] Mode " << cfg.sync_mode << ' '
                  << (cfg.sync_mode == 0 ? "free-run" : "external slave")
                  << ", "
                  << cfg.width << 'x' << cfg.height << '@' << cfg.fps
                  << " RGB=YUYV->MPP-MJPEG(q=" << cfg.jpeg_quality
                  << ") Depth=Z16\n"
                  << "[CONFIG] preallocated bundles=" << cfg.queue_capacity
                  << " estimated_pool=" << std::fixed << std::setprecision(1)
                  << pool_mib << " MiB capture_core=" << cfg.capture_core
                  << " encoder_core=" << cfg.encoder_core
                  << " writer_core=" << cfg.writer_core
                  << " camera_name=" << cfg.camera_name
                  << " write_depth=" << (cfg.write_depth ? "true" : "false")
                  << " control=" << cfg.control_mode << '\n';

        fs::create_directories(cfg.base_dir);
        rs2::context context;
        rs2::device device = select_device(context, cfg.serial);
        FramePool pool(cfg.queue_capacity, cfg.max_mjpeg_bytes, depth_bytes);
        std::atomic<bool> recording{false};
        WriterT writer(pool, cfg);
        EncoderT encoder(pool, cfg);
        CaptureT capture(context, device, pool, cfg, recording);
        SharedControl shared;

        writer.start();
        encoder.start();
        capture.start();

        const auto ready_deadline = std::chrono::steady_clock::now() +
            std::chrono::seconds(15);
        while (g_running && !capture.ready() && !capture.error() &&
               std::chrono::steady_clock::now() < ready_deadline)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        if (capture.error())
            throw std::runtime_error(capture.error_message());
        if (!capture.ready())
            throw std::runtime_error("camera pipeline readiness timeout");

        shared.set_state(CAMERA_STATE_IDLE);
        std::uint32_t current_session = 0;
        bool has_session = false;
        std::uint32_t last_request_seq =
            camera_shm_load_u32(&shared.get()->request_seq);

        auto wait_pipeline_drained = [&](int timeout_ms) -> bool
        {
            const auto deadline = std::chrono::steady_clock::now() +
                std::chrono::milliseconds(timeout_ms);
            while (std::chrono::steady_clock::now() < deadline)
            {
                if (pool.idle())
                {
                    return true;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
            return pool.idle();
        };

        auto start_session = [&](std::uint32_t session,
                                 std::string& error) -> bool
        {
            if (recording.load())
            {
                error = "SESSION_BUSY";
                return false;
            }
            if (has_session && current_session == session)
            {
                error = "SESSION_ALREADY_FINISHED";
                return false;
            }

            pool.clear_queued();
            shared.set_state(CAMERA_STATE_STARTING);
            std::string ep_path;
            if (!writer.open_session(session, capture.serial(),
                                     capture.depth_scale(), ep_path, error))
            {
                shared.set_state(CAMERA_STATE_ERROR);
                return false;
            }
            current_session = session;
            has_session = true;
            shared.set_ep_path(ep_path);
            shared.set_frames(0);
            recording.store(true);
            shared.set_state(CAMERA_STATE_RUNNING);
            std::cout << "[SESSION] START id=" << session << '\n';
            return true;
        };

        auto stop_session = [&](std::string& error) -> bool
        {
            if (!recording.load())
                return true;

            shared.set_state(CAMERA_STATE_STOPPING);
            recording.store(false);
            if (!wait_pipeline_drained(cfg.drain_timeout_ms))
            {
                std::cerr << "[WARN] drain timeout; dropping "
                          << pool.queued_size() << " queued bundles\n";
                pool.clear_queued();
                if (!wait_pipeline_drained(2000))
                {
                    error = "PIPELINE_DRAIN_TIMEOUT";
                    shared.set_state(CAMERA_STATE_ERROR);
                    return false;
                }
            }

            if (!writer.close_session(error))
            {
                shared.set_state(CAMERA_STATE_ERROR);
                return false;
            }
            shared.set_frames(writer.session_written());
            shared.set_state(CAMERA_STATE_IDLE);
            std::cout << "[SESSION] STOP id=" << current_session
                      << " frames=" << writer.session_written() << '\n';
            return true;
        };

        if (cfg.control_mode == "immediate")
        {
            std::string error;
            if (!start_session(1, error))
                throw std::runtime_error("immediate START failed: " + error);
        }

        auto last_stats = std::chrono::steady_clock::now();
        std::uint64_t last_seen = 0;
        std::uint64_t last_encoded = 0;
        std::uint64_t last_written = 0;
        std::uint64_t last_bytes = 0;

        while (g_running)
        {
            if (capture.error())
            {
                recording.store(false);
                shared.set_state(CAMERA_STATE_ERROR);
                shared.respond(last_request_seq,
                               CAMERA_RESULT_CAMERA_ERROR,
                               "CAMERA_CAPTURE_ERROR");
                break;
            }
            if (encoder.encoding_errors() > 0)
            {
                recording.store(false);
                shared.set_state(CAMERA_STATE_ERROR);
                shared.respond(last_request_seq,
                               CAMERA_RESULT_INTERNAL_ERROR,
                               "CAMERA_JPEG_ENCODER_ERROR");
                break;
            }
            if (writer.write_errors() > 0)
            {
                recording.store(false);
                shared.set_state(CAMERA_STATE_ERROR);
                shared.respond(last_request_seq,
                               CAMERA_RESULT_INTERNAL_ERROR,
                               "CAMERA_WRITE_ERROR");
                break;
            }

            if (cfg.control_mode == "shared_memory")
            {
                const std::uint32_t request_seq =
                    camera_shm_load_u32(&shared.get()->request_seq);
                if (request_seq != last_request_seq)
                {
                    last_request_seq = request_seq;
                    const auto command = static_cast<CameraControlCommand>(
                        camera_shm_load_u32(&shared.get()->command));
                    const std::uint32_t requested_session =
                        camera_shm_load_u32(&shared.get()->session_id);
                    std::string error;

                    if (command == CAMERA_COMMAND_START)
                    {
                        if (recording.load() &&
                            requested_session == current_session)
                        {
                            shared.respond(request_seq, CAMERA_RESULT_OK, "");
                        }
                        else if (start_session(requested_session, error))
                        {
                            shared.respond(request_seq, CAMERA_RESULT_OK, "");
                        }
                        else
                        {
                            shared.respond(request_seq, CAMERA_RESULT_BUSY, error);
                        }
                    }
                    else if (command == CAMERA_COMMAND_STOP)
                    {
                        if (recording.load() &&
                            requested_session != current_session)
                        {
                            shared.respond(request_seq,
                                CAMERA_RESULT_SESSION_MISMATCH,
                                "SESSION_MISMATCH");
                        }
                        else if (stop_session(error))
                        {
                            shared.respond(request_seq, CAMERA_RESULT_OK, "");
                        }
                        else
                        {
                            shared.respond(request_seq,
                                CAMERA_RESULT_INTERNAL_ERROR, error);
                        }
                    }
                    else if (command == CAMERA_COMMAND_STATUS)
                    {
                        shared.set_frames(writer.session_written());
                        shared.respond(request_seq, CAMERA_RESULT_OK, "");
                    }
                    else
                    {
                        shared.respond(request_seq,
                            CAMERA_RESULT_INVALID_COMMAND,
                            "INVALID_COMMAND");
                    }
                }
            }

            if (recording.load())
                shared.set_frames(writer.session_written());

            const auto now = std::chrono::steady_clock::now();
            const double elapsed = std::chrono::duration<double>(
                now - last_stats).count();
            if (elapsed >= 1.0)
            {
                const std::uint64_t seen = capture.frames_seen();
                const std::uint64_t encoded = encoder.frames_encoded();
                const std::uint64_t written = writer.session_written();
                const std::uint64_t bytes = writer.bytes_written();
                const std::uint64_t written_delta = written >= last_written
                    ? written - last_written
                    : written;
                std::cout << "[STATS] rec="
                          << (recording.load() ? "ON" : "OFF")
                          << " capture_fps=" << std::setprecision(1)
                          << (seen - last_seen) / elapsed
                          << " encode_fps=" << (encoded - last_encoded) / elapsed
                          << " encode_ms=" << std::setprecision(2)
                          << encoder.average_encode_ms()
                          << " write_fps=" << written_delta / elapsed
                          << " write_MiB_s=" << std::setprecision(2)
                          << (bytes - last_bytes) / elapsed / 1024.0 / 1024.0
                          << " enc_q=" << pool.encode_size()
                          << " write_q=" << pool.write_size()
                          << " peak=" << pool.peak_total()
                          << " queue_drop=" << capture.queue_drops()
                          << " oversize_drop=" << capture.oversize_drops()
                          << " rgb_domain/depth_domain logged in index"
                          << '\n';
                last_seen = seen;
                last_encoded = encoded;
                last_written = written;
                last_bytes = bytes;
                last_stats = now;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        recording.store(false);
        capture.stop();
        capture.join();
        if (encoder.encoding_errors() > 0 || writer.write_errors() > 0)
            pool.clear_queued();
        if (!wait_pipeline_drained(cfg.drain_timeout_ms))
            pool.clear_queued();
        std::string close_error;
        writer.close_session(close_error);
        encoder.stop();
        encoder.join();
        writer.stop();
        writer.join();
        if (!close_error.empty())
            std::cerr << "[WARN] close error: " << close_error << '\n';
        return capture.error() || encoder.encoding_errors() > 0 ||
            writer.write_errors() > 0 ? 1 : 0;
    }
    catch (const rs2::error& error)
    {
        std::cerr << "[RealSense ERROR] " << error.what()
                  << " function=" << error.get_failed_function()
                  << " args=" << error.get_failed_args() << '\n';
        return 1;
    }
    catch (const std::exception& error)
    {
        std::cerr << "[ERROR] " << error.what() << '\n';
        return 1;
    }
}

namespace
{

class ChunkWriter
{
public:
    ChunkWriter(FramePool& pool, const AppConfig& cfg)
        : pool_(pool), cfg_(cfg)
    {
    }

    ~ChunkWriter()
    {
        stop();
        join();
    }

    void start()
    {
        thread_ = std::thread(&ChunkWriter::run, this);
    }

    void stop()
    {
        running_.store(false);
    }

    void join()
    {
        if (thread_.joinable())
            thread_.join();
    }

    bool open_session(std::uint32_t session_id, const std::string& serial,
                      float depth_scale, std::string& ep_path,
                      std::string& error)
    {
        std::lock_guard<std::mutex> lock(io_mutex_);
        try
        {
            if (session_open_)
                throw std::runtime_error("a session is already open");

            const fs::path episode_directory = fs::path(cfg_.base_dir) /
                ("ep_" + local_timestamp());
            const fs::path directory = episode_directory / cfg_.camera_name;
            if (fs::exists(directory))
                throw std::runtime_error("camera output directory already exists");
            fs::create_directories(directory);

            write_manifest(directory, session_id, serial, depth_scale);
            fs::copy_file(cfg_.config_path, directory / "capture_config.yaml",
                          fs::copy_options::overwrite_existing);

            session_dir_ = directory;
            session_id_ = session_id;
            serial_ = serial;
            depth_scale_ = depth_scale;
            chunk_id_ = 0;
            frames_in_chunk_ = 0;
            session_written_.store(0);
            session_open_ = true;
            ep_path = directory.string();
            std::cout << "[WRITER] session opened: " << ep_path << '\n';
            return true;
        }
        catch (const std::exception& exception)
        {
            error = exception.what();
            session_open_ = false;
            return false;
        }
    }

    bool wait_drained(int timeout_ms)
    {
        const auto deadline = std::chrono::steady_clock::now() +
            std::chrono::milliseconds(timeout_ms);
        while (std::chrono::steady_clock::now() < deadline)
        {
            if (pool_.write_size() == 0 && !in_flight_.load())
                return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        return pool_.write_size() == 0 && !in_flight_.load();
    }

    bool close_session(std::string& error)
    {
        std::lock_guard<std::mutex> lock(io_mutex_);
        try
        {
            chunks_.close(cfg_.fdatasync_on_chunk_close);
            session_open_ = false;
            session_dir_.clear();
            std::cout << "[WRITER] session closed; frames="
                      << session_written_.load() << '\n';
            return true;
        }
        catch (const std::exception& exception)
        {
            error = exception.what();
            session_open_ = false;
            return false;
        }
    }

    std::uint64_t session_written() const
    {
        return session_written_.load();
    }

    std::uint64_t write_errors() const
    {
        return write_errors_.load();
    }

    std::uint64_t bytes_written() const
    {
        return bytes_written_.load();
    }

    bool in_flight() const { return in_flight_.load(); }

private:
    void write_manifest(const fs::path& directory, std::uint32_t session_id,
                        const std::string& serial, float depth_scale)
    {
        std::ofstream file(directory / "manifest.yaml",
                           std::ios::out | std::ios::trunc);
        if (!file)
            throw std::runtime_error("cannot create manifest.yaml");

        file << "format_version: 1\n"
             << "session_id: " << session_id << "\n"
             << "camera_serial: \"" << serial << "\"\n"
             << "camera_name: \"" << cfg_.camera_name << "\"\n"
             << "inter_cam_sync_mode: " << cfg_.sync_mode << "\n"
             << "width: " << cfg_.width << "\n"
             << "height: " << cfg_.height << "\n"
             << "fps: " << cfg_.fps << "\n"
             << "rgb_input_format: YUYV\n"
             << "rgb_format: MJPEG\n"
             << "jpeg_encoder: rockchip_mpp\n"
             << "jpeg_quality: " << cfg_.jpeg_quality << "\n"
             << "depth_format: Z16_LE\n"
             << "depth_written: " << (cfg_.write_depth ? "true" : "false")
             << "\n"
             << std::setprecision(9)
             << "depth_scale: " << depth_scale << "\n"
             << "chunk_frames: " << cfg_.chunk_frames << "\n"
             << "index_header_bytes: " << sizeof(IndexHeader) << "\n"
             << "index_record_bytes: " << sizeof(IndexRecord) << "\n"
             << "created_unix_ns: " << unix_now_ns() << "\n";
        if (!file)
            throw std::runtime_error("manifest.yaml write failed");
    }

    void ensure_chunk()
    {
        if (chunks_.is_open() && frames_in_chunk_ < cfg_.chunk_frames)
            return;
        if (chunks_.is_open())
        {
            chunks_.close(cfg_.fdatasync_on_chunk_close);
            ++chunk_id_;
        }
        chunks_.open(session_dir_, chunk_id_, session_id_, cfg_, depth_scale_);
        frames_in_chunk_ = 0;
    }

    void run()
    {
        try
        {
            bind_current_thread(cfg_.writer_core, "writer");
        }
        catch (const std::exception& exception)
        {
            std::cerr << "[WRITER ERROR] " << exception.what() << '\n';
            fatal_error_ = exception.what();
            write_errors_.fetch_add(1);
            running_.store(false);
        }

        while (running_.load() || pool_.write_size() > 0)
        {
            FrameBundle* frame = nullptr;
            if (!pool_.pop_for_write(frame, 100))
                continue;

            in_flight_.store(true);
            try
            {
                std::lock_guard<std::mutex> lock(io_mutex_);
                if (session_open_)
                {
                    ensure_chunk();
                    chunks_.append(*frame);
                    ++frames_in_chunk_;
                    session_written_.fetch_add(1);
                    bytes_written_.fetch_add(
                        static_cast<std::uint64_t>(frame->rgb_bytes) +
                        (cfg_.write_depth ? frame->depth_bytes : 0U) +
                        sizeof(IndexRecord));

                    // write(2) already transfers bytes into the kernel page
                    // cache. There is no extra userspace buffer to flush here.
                    // Durability is enforced once per completed chunk.
                }
            }
            catch (const std::exception& exception)
            {
                write_errors_.fetch_add(1);
                fatal_error_ = exception.what();
                std::cerr << "[WRITER ERROR] " << exception.what() << '\n';
            }
            pool_.release_after_write(frame);
            in_flight_.store(false);
        }
    }

    FramePool& pool_;
    const AppConfig& cfg_;
    std::thread thread_;
    std::atomic<bool> running_{true};
    std::atomic<bool> in_flight_{false};
    mutable std::mutex io_mutex_;
    bool session_open_ = false;
    fs::path session_dir_;
    std::uint32_t session_id_ = 0;
    std::string serial_;
    float depth_scale_ = 0.001F;
    std::uint32_t chunk_id_ = 0;
    std::uint32_t frames_in_chunk_ = 0;
    ChunkFiles chunks_;
    std::atomic<std::uint64_t> session_written_{0};
    std::atomic<std::uint64_t> bytes_written_{0};
    std::atomic<std::uint64_t> write_errors_{0};
    std::string fatal_error_;
};

class MppJpegEncoder
{
public:
    MppJpegEncoder(FramePool& pool, const AppConfig& cfg)
        : pool_(pool), cfg_(cfg)
    {
        try
        {
            initialize();
        }
        catch (...)
        {
            cleanup();
            throw;
        }
    }

    ~MppJpegEncoder()
    {
        stop();
        join();
        cleanup();
    }

    void start()
    {
        thread_ = std::thread(&MppJpegEncoder::run, this);
    }

    void stop() { running_.store(false); }

    void join()
    {
        if (thread_.joinable())
            thread_.join();
    }

    bool in_flight() const { return in_flight_.load(); }
    std::uint64_t frames_encoded() const { return frames_encoded_.load(); }
    std::uint64_t encoding_errors() const { return encoding_errors_.load(); }

    double average_encode_ms() const
    {
        const std::uint64_t count = frames_encoded_.load();
        return count == 0 ? 0.0 :
            static_cast<double>(total_encode_ns_.load()) /
                static_cast<double>(count) / 1.0e6;
    }

private:
    static std::size_t align_up(std::size_t value, std::size_t alignment)
    {
        return (value + alignment - 1U) & ~(alignment - 1U);
    }

    static void require_mpp(MPP_RET result, const char* operation)
    {
        if (result != MPP_OK)
        {
            throw std::runtime_error(
                std::string(operation) + " failed, MPP_RET=" +
                std::to_string(static_cast<int>(result)));
        }
    }

    void set_cfg_s32(const char* key, RK_S32 value)
    {
        require_mpp(mpp_enc_cfg_set_s32(enc_cfg_, key, value), key);
    }

    void initialize()
    {
        packed_yuyv_bytes_ = static_cast<std::size_t>(cfg_.width) *
            static_cast<std::size_t>(cfg_.height) * 2U;

        // Match mpi_enc_width_default_stride() for format 8. For packed YUYV
        // MPP expresses horizontal stride in bytes: ALIGN(width, 8) * 2.
        mpp_hor_stride_ = static_cast<RK_U32>(align_up(
            static_cast<std::size_t>(cfg_.width), 8U) * 2U);
        mpp_ver_stride_ = static_cast<RK_U32>(align_up(
            static_cast<std::size_t>(cfg_.height), 16U));
        const std::size_t input_buffer_bytes =
            align_up(mpp_hor_stride_, 64U) *
            align_up(mpp_ver_stride_, 64U);

        require_mpp(mpp_create(&ctx_, &mpi_), "mpp_create");
        MppPollType timeout = MPP_POLL_BLOCK;
        require_mpp(mpi_->control(ctx_, MPP_SET_OUTPUT_TIMEOUT, &timeout),
                    "MPP_SET_OUTPUT_TIMEOUT");
        require_mpp(mpp_init(ctx_, MPP_CTX_ENC, MPP_VIDEO_CodingMJPEG),
                    "mpp_init(MJPEG encoder)");
        require_mpp(mpp_enc_cfg_init(&enc_cfg_), "mpp_enc_cfg_init");
        require_mpp(mpi_->control(ctx_, MPP_ENC_GET_CFG, enc_cfg_),
                    "MPP_ENC_GET_CFG");

        set_cfg_s32("prep:width", cfg_.width);
        set_cfg_s32("prep:height", cfg_.height);
        set_cfg_s32("prep:hor_stride", static_cast<RK_S32>(mpp_hor_stride_));
        set_cfg_s32("prep:ver_stride", static_cast<RK_S32>(mpp_ver_stride_));
        set_cfg_s32("prep:format", MPP_FMT_YUV422_YUYV);
        set_cfg_s32("rc:mode", MPP_ENC_RC_MODE_FIXQP);
        set_cfg_s32("rc:fps_in_flex", 0);
        set_cfg_s32("rc:fps_in_num", cfg_.fps);
        set_cfg_s32("rc:fps_in_denom", 1);
        set_cfg_s32("rc:fps_out_flex", 0);
        set_cfg_s32("rc:fps_out_num", cfg_.fps);
        set_cfg_s32("rc:fps_out_denom", 1);
        set_cfg_s32("jpeg:q_factor", cfg_.jpeg_quality);
        set_cfg_s32("jpeg:qf_min", 1);
        set_cfg_s32("jpeg:qf_max", 99);
        require_mpp(mpi_->control(ctx_, MPP_ENC_SET_CFG, enc_cfg_),
                    "MPP_ENC_SET_CFG");

        const auto buffer_type = static_cast<MppBufferType>(
            MPP_BUFFER_TYPE_DRM | MPP_BUFFER_FLAGS_CACHABLE);
        require_mpp(mpp_buffer_group_get_internal(&buffer_group_, buffer_type),
                    "mpp_buffer_group_get_internal");
        require_mpp(mpp_buffer_get(buffer_group_, &input_buffer_,
                                   input_buffer_bytes),
                    "mpp_buffer_get(input)");
        require_mpp(mpp_buffer_get(buffer_group_, &output_buffer_,
                                   cfg_.max_mjpeg_bytes),
                    "mpp_buffer_get(output)");

        std::cout << "[ENCODER] Rockchip MPP MJPEG ready input=YUYV"
                  << " quality=" << cfg_.jpeg_quality
                  << " input_buffer=" << input_buffer_bytes
                  << " output_buffer=" << cfg_.max_mjpeg_bytes << '\n';
    }

    void cleanup() noexcept
    {
        if (ctx_ != nullptr)
        {
            mpp_destroy(ctx_);
            ctx_ = nullptr;
            mpi_ = nullptr;
        }
        if (enc_cfg_ != nullptr)
        {
            mpp_enc_cfg_deinit(enc_cfg_);
            enc_cfg_ = nullptr;
        }
        if (output_buffer_ != nullptr)
        {
            mpp_buffer_put(output_buffer_);
            output_buffer_ = nullptr;
        }
        if (input_buffer_ != nullptr)
        {
            mpp_buffer_put(input_buffer_);
            input_buffer_ = nullptr;
        }
        if (buffer_group_ != nullptr)
        {
            mpp_buffer_group_put(buffer_group_);
            buffer_group_ = nullptr;
        }
    }

    void encode(FrameBundle& bundle)
    {
        if (bundle.rgb_bytes != packed_yuyv_bytes_)
        {
            throw std::runtime_error(
                "unexpected packed YUYV byte count: " +
                std::to_string(bundle.rgb_bytes));
        }

        void* input = mpp_buffer_get_ptr(input_buffer_);
        if (input == nullptr)
            throw std::runtime_error("MPP input buffer has no CPU mapping");
        require_mpp(mpp_buffer_sync_begin(input_buffer_),
                    "mpp_buffer_sync_begin(input)");
        const std::size_t source_row_bytes =
            static_cast<std::size_t>(cfg_.width) * 2U;
        const std::size_t mpp_row_bytes =
            static_cast<std::size_t>(mpp_hor_stride_);
        if (source_row_bytes == mpp_row_bytes)
        {
            std::memcpy(input, bundle.rgb.data(), packed_yuyv_bytes_);
        }
        else
        {
            auto* destination = static_cast<std::uint8_t*>(input);
            for (int row = 0; row < cfg_.height; ++row)
            {
                std::memcpy(destination + row * mpp_row_bytes,
                            bundle.rgb.data() + row * source_row_bytes,
                            source_row_bytes);
                std::memset(destination + row * mpp_row_bytes + source_row_bytes,
                            0, mpp_row_bytes - source_row_bytes);
            }
        }
        require_mpp(mpp_buffer_sync_end(input_buffer_),
                    "mpp_buffer_sync_end(input)");

        MppFrame frame = nullptr;
        MppPacket packet = nullptr;
        try
        {
            require_mpp(mpp_frame_init(&frame), "mpp_frame_init");
            mpp_frame_set_width(frame, static_cast<RK_U32>(cfg_.width));
            mpp_frame_set_height(frame, static_cast<RK_U32>(cfg_.height));
            mpp_frame_set_hor_stride(frame, mpp_hor_stride_);
            mpp_frame_set_ver_stride(frame, mpp_ver_stride_);
            mpp_frame_set_fmt(frame, MPP_FMT_YUV422_YUYV);
            mpp_frame_set_pts(frame, static_cast<RK_S64>(bundle.sequence));
            mpp_frame_set_buffer(frame, input_buffer_);

            require_mpp(mpp_packet_init_with_buffer(&packet, output_buffer_),
                        "mpp_packet_init_with_buffer");
            mpp_packet_set_length(packet, 0);
            MppMeta meta = mpp_frame_get_meta(frame);
            if (meta == nullptr)
                throw std::runtime_error("mpp_frame_get_meta returned null");
            require_mpp(mpp_meta_set_packet(meta, KEY_OUTPUT_PACKET, packet),
                        "mpp_meta_set_packet(KEY_OUTPUT_PACKET)");

            require_mpp(mpi_->encode_put_frame(ctx_, frame),
                        "encode_put_frame");
            mpp_frame_deinit(&frame);
            require_mpp(mpi_->encode_get_packet(ctx_, &packet),
                        "encode_get_packet");
            if (packet == nullptr)
                throw std::runtime_error("MPP returned no JPEG packet");

            const std::size_t jpeg_bytes = mpp_packet_get_length(packet);
            const auto* jpeg = static_cast<const std::uint8_t*>(
                mpp_packet_get_pos(packet));
            if (jpeg == nullptr || jpeg_bytes < 4U ||
                jpeg_bytes > bundle.rgb.size())
            {
                throw std::runtime_error(
                    "invalid MPP JPEG packet length: " +
                    std::to_string(jpeg_bytes));
            }

            require_mpp(mpp_buffer_sync_begin(output_buffer_),
                        "mpp_buffer_sync_begin(output)");
            const bool jpeg_markers_ok = jpeg[0] == 0xffU && jpeg[1] == 0xd8U &&
                jpeg[jpeg_bytes - 2U] == 0xffU &&
                jpeg[jpeg_bytes - 1U] == 0xd9U;
            if (jpeg_markers_ok)
                std::memcpy(bundle.rgb.data(), jpeg, jpeg_bytes);
            require_mpp(mpp_buffer_sync_end(output_buffer_),
                        "mpp_buffer_sync_end(output)");
            if (!jpeg_markers_ok)
                throw std::runtime_error("MPP packet is not a complete JPEG");

            bundle.rgb_bytes = static_cast<std::uint32_t>(jpeg_bytes);
            mpp_packet_deinit(&packet);
        }
        catch (...)
        {
            if (frame != nullptr)
                mpp_frame_deinit(&frame);
            if (packet != nullptr)
                mpp_packet_deinit(&packet);
            throw;
        }
    }

    void run()
    {
        try
        {
            bind_current_thread(cfg_.encoder_core, "mpp-encoder");
        }
        catch (const std::exception& exception)
        {
            std::cerr << "[ENCODER ERROR] " << exception.what() << '\n';
            encoding_errors_.fetch_add(1);
            running_.store(false);
            return;
        }

        while (running_.load() || pool_.encode_size() > 0)
        {
            FrameBundle* frame = nullptr;
            if (!pool_.pop_for_encode(frame, 100))
                continue;

            in_flight_.store(true);
            try
            {
                const auto started = std::chrono::steady_clock::now();
                encode(*frame);
                const auto elapsed = std::chrono::duration_cast<
                    std::chrono::nanoseconds>(
                    std::chrono::steady_clock::now() - started).count();
                total_encode_ns_.fetch_add(static_cast<std::uint64_t>(elapsed));
                frames_encoded_.fetch_add(1);
                pool_.submit_for_write(frame);
            }
            catch (const std::exception& exception)
            {
                encoding_errors_.fetch_add(1);
                std::cerr << "[ENCODER ERROR] " << exception.what() << '\n';
                pool_.release_after_encode(frame);
                running_.store(false);
                in_flight_.store(false);
                break;
            }
            in_flight_.store(false);
        }
    }

    FramePool& pool_;
    const AppConfig& cfg_;
    std::thread thread_;
    std::atomic<bool> running_{true};
    std::atomic<bool> in_flight_{false};
    std::atomic<std::uint64_t> frames_encoded_{0};
    std::atomic<std::uint64_t> encoding_errors_{0};
    std::atomic<std::uint64_t> total_encode_ns_{0};
    std::size_t packed_yuyv_bytes_ = 0;
    RK_U32 mpp_hor_stride_ = 0;
    RK_U32 mpp_ver_stride_ = 0;
    MppCtx ctx_ = nullptr;
    MppApi* mpi_ = nullptr;
    MppEncCfg enc_cfg_ = nullptr;
    MppBufferGroup buffer_group_ = nullptr;
    MppBuffer input_buffer_ = nullptr;
    MppBuffer output_buffer_ = nullptr;
};

bool sensor_has_stream(const rs2::sensor& sensor, rs2_stream stream)
{
    for (const rs2::stream_profile& profile : sensor.get_stream_profiles())
    {
        if (profile.stream_type() == stream)
            return true;
    }
    return false;
}

rs2::device select_device(rs2::context& context, const std::string& serial)
{
    rs2::device_list devices = context.query_devices();
    if (devices.size() == 0)
        throw std::runtime_error("no RealSense device found");

    if (serial.empty() && devices.size() != 1)
    {
        throw std::runtime_error(
            "device.serial is empty but attached device count is " +
            std::to_string(devices.size()));
    }

    for (rs2::device device : devices)
    {
        const std::string candidate =
            device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        if (serial.empty() || candidate == serial)
            return device;
    }
    throw std::runtime_error("configured RealSense serial was not found: " + serial);
}

bool has_video_profile(const rs2::device& device, rs2_stream stream,
                       rs2_format format, int width, int height, int fps)
{
    for (const rs2::sensor& sensor : device.query_sensors())
    {
        for (const rs2::stream_profile& profile : sensor.get_stream_profiles())
        {
            if (profile.stream_type() != stream || profile.format() != format ||
                profile.fps() != fps || !profile.is<rs2::video_stream_profile>())
            {
                continue;
            }
            const auto video = profile.as<rs2::video_stream_profile>();
            if (video.width() == width && video.height() == height)
                return true;
        }
    }
    return false;
}

void print_stream_profiles(const rs2::device& device, rs2_stream stream)
{
    std::cerr << "Available " << rs2_stream_to_string(stream)
              << " video profiles:\n";
    for (const rs2::sensor& sensor : device.query_sensors())
    {
        for (const rs2::stream_profile& profile : sensor.get_stream_profiles())
        {
            if (profile.stream_type() != stream ||
                !profile.is<rs2::video_stream_profile>())
            {
                continue;
            }
            const auto video = profile.as<rs2::video_stream_profile>();
            std::cerr << "  " << video.width() << 'x' << video.height()
                      << '@' << profile.fps() << ' '
                      << rs2_format_to_string(profile.format()) << '\n';
        }
    }
}

void set_option_if_supported(rs2::sensor& sensor, rs2_option option,
                             float value, const char* label)
{
    if (!sensor.supports(option))
        return;
    sensor.set_option(option, value);
    std::cout << "[CAMERA] " << label << '=' << value << '\n';
}

float configure_device(rs2::device& device, const AppConfig& cfg)
{
    bool sync_configured = false;
    float depth_scale = 0.001F;

    for (rs2::sensor sensor : device.query_sensors())
    {
        const bool is_depth = sensor.is<rs2::depth_sensor>();
        const bool is_color = sensor_has_stream(sensor, RS2_STREAM_COLOR);

        if (is_depth)
        {
            auto depth_sensor = sensor.as<rs2::depth_sensor>();
            depth_scale = depth_sensor.get_depth_scale();
            if (!sensor.supports(RS2_OPTION_INTER_CAM_SYNC_MODE))
            {
                throw std::runtime_error(
                    "depth sensor does not support inter-camera sync");
            }
            sensor.set_option(RS2_OPTION_INTER_CAM_SYNC_MODE,
                              static_cast<float>(cfg.sync_mode));
            const float sync_mode_readback =
                sensor.get_option(RS2_OPTION_INTER_CAM_SYNC_MODE);
            sync_configured = true;
            std::cout << "[SYNC] Stereo Module requested Mode "
                      << cfg.sync_mode << ", readback=" << sync_mode_readback
                      << " ("
                      << (cfg.sync_mode == 0 ? "free-run" : "external slave")
                      << ")\n";
        }

        if (cfg.disable_auto_exposure)
        {
            set_option_if_supported(sensor, RS2_OPTION_ENABLE_AUTO_EXPOSURE,
                                    0.0F, "auto_exposure");
            set_option_if_supported(sensor, RS2_OPTION_EXPOSURE,
                                    is_color ? cfg.color_exposure
                                             : cfg.depth_exposure,
                                    "exposure");
            set_option_if_supported(sensor, RS2_OPTION_GAIN,
                                    is_color ? cfg.color_gain : cfg.depth_gain,
                                    "gain");
        }

        if (is_color && cfg.disable_auto_exposure_priority)
        {
            set_option_if_supported(sensor,
                                    RS2_OPTION_AUTO_EXPOSURE_PRIORITY,
                                    0.0F, "auto_exposure_priority");
        }
        set_option_if_supported(sensor, RS2_OPTION_GLOBAL_TIME_ENABLED,
                                cfg.global_time_enabled ? 1.0F : 0.0F,
                                "global_time_enabled");
    }

    if (!sync_configured)
        throw std::runtime_error(
            "failed to configure inter-camera sync mode on depth sensor");
    return depth_scale;
}

class CaptureWorker
{
public:
    CaptureWorker(rs2::context& context, rs2::device device,
                  FramePool& pool, const AppConfig& cfg,
                  std::atomic<bool>& recording)
        : context_(context), device_(std::move(device)), pool_(pool),
          cfg_(cfg), recording_(recording)
    {
        serial_ = device_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    }

    ~CaptureWorker()
    {
        stop();
        join();
    }

    void start()
    {
        thread_ = std::thread(&CaptureWorker::run, this);
    }

    void stop() { running_.store(false); }

    void join()
    {
        if (thread_.joinable())
            thread_.join();
    }

    bool ready() const { return ready_.load(); }
    bool error() const { return error_.load(); }
    std::string error_message() const
    {
        std::lock_guard<std::mutex> lock(error_mutex_);
        return error_message_;
    }
    const std::string& serial() const { return serial_; }
    float depth_scale() const { return depth_scale_; }
    std::uint64_t frames_seen() const { return frames_seen_.load(); }
    std::uint64_t accepted() const { return accepted_.load(); }
    std::uint64_t queue_drops() const { return queue_drops_.load(); }
    std::uint64_t oversize_drops() const { return oversize_drops_.load(); }

private:
    void set_error(const std::string& value)
    {
        {
            std::lock_guard<std::mutex> lock(error_mutex_);
            error_message_ = value;
        }
        error_.store(true);
    }

    void copy_depth_packed(const rs2::depth_frame& depth_frame,
                           FrameBundle& item)
    {
        const std::size_t row_bytes =
            static_cast<std::size_t>(cfg_.width) * sizeof(std::uint16_t);
        const std::size_t packed_bytes =
            row_bytes * static_cast<std::size_t>(cfg_.height);
        if (packed_bytes > item.depth.size())
            throw std::runtime_error("preallocated depth buffer is too small");

        const auto* source = static_cast<const std::uint8_t*>(
            depth_frame.get_data());
        const std::size_t stride = depth_frame.get_stride_in_bytes();
        if (stride == row_bytes)
        {
            std::memcpy(item.depth.data(), source, packed_bytes);
        }
        else
        {
            for (int row = 0; row < cfg_.height; ++row)
            {
                std::memcpy(item.depth.data() + row * row_bytes,
                            source + row * stride, row_bytes);
            }
        }
        item.depth_bytes = static_cast<std::uint32_t>(packed_bytes);
    }

    void copy_yuyv_packed(const rs2::video_frame& rgb_frame,
                          FrameBundle& item)
    {
        const std::size_t row_bytes =
            static_cast<std::size_t>(cfg_.width) * 2U;
        const std::size_t packed_bytes =
            row_bytes * static_cast<std::size_t>(cfg_.height);
        if (packed_bytes > item.rgb.size())
            throw std::runtime_error("preallocated RGB buffer is too small");

        const auto* source = static_cast<const std::uint8_t*>(
            rgb_frame.get_data());
        const std::size_t stride = rgb_frame.get_stride_in_bytes();
        if (stride < row_bytes)
            throw std::runtime_error("invalid YUYV source stride");
        if (stride == row_bytes)
        {
            std::memcpy(item.rgb.data(), source, packed_bytes);
        }
        else
        {
            for (int row = 0; row < cfg_.height; ++row)
            {
                std::memcpy(item.rgb.data() + row * row_bytes,
                            source + row * stride, row_bytes);
            }
        }
        item.rgb_bytes = static_cast<std::uint32_t>(packed_bytes);
    }

    void run()
    {
        try
        {
            bind_current_thread(cfg_.capture_core, "capture");

            if (!has_video_profile(device_, RS2_STREAM_COLOR, RS2_FORMAT_YUYV,
                                   cfg_.width, cfg_.height, cfg_.fps))
            {
                print_stream_profiles(device_, RS2_STREAM_COLOR);
                throw std::runtime_error(
                    "required native YUYV profile is not supported");
            }
            if (!has_video_profile(device_, RS2_STREAM_DEPTH, RS2_FORMAT_Z16,
                                   cfg_.width, cfg_.height, cfg_.fps))
            {
                print_stream_profiles(device_, RS2_STREAM_DEPTH);
                throw std::runtime_error(
                    "required Z16 profile is not supported");
            }

            depth_scale_ = configure_device(device_, cfg_);

            rs2::config pipeline_config;
            pipeline_config.enable_device(serial_);
            pipeline_config.enable_stream(
                RS2_STREAM_COLOR, cfg_.width, cfg_.height,
                RS2_FORMAT_YUYV, cfg_.fps);
            pipeline_config.enable_stream(
                RS2_STREAM_DEPTH, cfg_.width, cfg_.height,
                RS2_FORMAT_Z16, cfg_.fps);

            rs2::pipeline pipeline(context_);
            pipeline.start(pipeline_config);
            ready_.store(true);
            std::cout << "[CAPTURE] ready serial=" << serial_
                      << " RGB=YUYV (MPP output MJPEG) Depth=Z16 "
                      << cfg_.width << 'x' << cfg_.height << '@' << cfg_.fps
                      << " depth_scale=" << std::defaultfloat
                      << std::setprecision(9) << depth_scale_
                      << std::fixed << std::setprecision(1) << '\n';

            while (running_.load() && g_running)
            {
                rs2::frameset frames;
                if (!pipeline.poll_for_frames(&frames))
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    continue;
                }
                const std::int64_t host_receive_ns = unix_now_ns();

                rs2::video_frame rgb_frame = frames.get_color_frame();
                rs2::depth_frame depth_frame = frames.get_depth_frame();
                if (!rgb_frame || !depth_frame)
                    continue;

                frames_seen_.fetch_add(1);
                if (!recording_.load())
                    continue;

                const std::uint64_t sequence = input_sequence_.fetch_add(1) + 1;
                FrameBundle* item = pool_.try_acquire();
                if (item == nullptr)
                {
                    queue_drops_.fetch_add(1);
                    continue;
                }

                try
                {
                    copy_yuyv_packed(rgb_frame, *item);
                    copy_depth_packed(depth_frame, *item);

                    item->sequence = sequence;
                    item->rgb_frame_number = rgb_frame.get_frame_number();
                    item->depth_frame_number = depth_frame.get_frame_number();
                    item->host_receive_unix_ns = host_receive_ns;
                    item->rgb_rs_timestamp_ms = rgb_frame.get_timestamp();
                    item->depth_rs_timestamp_ms = depth_frame.get_timestamp();
                    item->rgb_sensor_timestamp = metadata_or_minus_one(
                        rgb_frame, RS2_FRAME_METADATA_SENSOR_TIMESTAMP);
                    item->depth_sensor_timestamp = metadata_or_minus_one(
                        depth_frame, RS2_FRAME_METADATA_SENSOR_TIMESTAMP);
                    item->rgb_timestamp_domain = static_cast<std::uint32_t>(
                        rgb_frame.get_frame_timestamp_domain());
                    item->depth_timestamp_domain = static_cast<std::uint32_t>(
                        depth_frame.get_frame_timestamp_domain());

                    // STOP may arrive while this bundle is being copied.
                    if (!recording_.load())
                    {
                        pool_.release(item);
                        continue;
                    }
                    pool_.submit_for_encode(item);
                    accepted_.fetch_add(1);
                }
                catch (...)
                {
                    pool_.release(item);
                    throw;
                }
            }

            pipeline.stop();
        }
        catch (const std::exception& exception)
        {
            set_error(exception.what());
            std::cerr << "[CAPTURE ERROR] " << exception.what() << '\n';
        }
    }

    rs2::context& context_;
    rs2::device device_;
    FramePool& pool_;
    const AppConfig& cfg_;
    std::atomic<bool>& recording_;
    std::thread thread_;
    std::string serial_;
    float depth_scale_ = 0.001F;
    std::atomic<bool> running_{true};
    std::atomic<bool> ready_{false};
    std::atomic<bool> error_{false};
    mutable std::mutex error_mutex_;
    std::string error_message_;
    std::atomic<std::uint64_t> frames_seen_{0};
    std::atomic<std::uint64_t> input_sequence_{0};
    std::atomic<std::uint64_t> accepted_{0};
    std::atomic<std::uint64_t> queue_drops_{0};
    std::atomic<std::uint64_t> oversize_drops_{0};
};

} // namespace

int main(int argc, char** argv)
{
    return run_main<ChunkWriter, MppJpegEncoder, CaptureWorker>(argc, argv);
}
