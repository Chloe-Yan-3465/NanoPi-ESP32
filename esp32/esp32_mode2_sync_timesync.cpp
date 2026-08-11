#include <Arduino.h>

#include <BLE2902.h>
#include <BLEAdvertisedDevice.h>
#include <BLEClient.h>
#include <BLEDevice.h>
#include <BLERemoteCharacteristic.h>
#include <BLERemoteService.h>
#include <BLEScan.h>
#include <BLEServer.h>
#include <BLEUtils.h>

#include <driver/gpio.h>
#include <driver/rmt.h>
#include <esp_crc.h>
#include <esp_timer.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <string>

#if defined(MODE2_ROLE_COORDINATOR) && defined(MODE2_ROLE_WEARABLE)
#error "Select exactly one firmware role"
#endif
#if !defined(MODE2_ROLE_COORDINATOR) && !defined(MODE2_ROLE_WEARABLE)
#error "Define MODE2_ROLE_COORDINATOR or MODE2_ROLE_WEARABLE"
#endif
#if defined(MODE2_ROLE_WEARABLE) && !defined(MODE2_NODE_ID)
#error "Wearable firmware requires MODE2_NODE_ID=1, 2, or 3"
#endif
#ifndef MODE2_NODE_COUNT
#define MODE2_NODE_COUNT 3
#endif

namespace {

constexpr char kServiceUuid[] = "5d6f0001-4f3c-4f59-a9f2-36f36a7c1000";
constexpr char kCommandUuid[] = "5d6f0002-4f3c-4f59-a9f2-36f36a7c1000";
constexpr char kStatusUuid[]  = "5d6f0003-4f3c-4f59-a9f2-36f36a7c1000";

constexpr std::uint32_t kWireMagic = 0x324D5352U;  // "RSM2"
constexpr std::uint8_t kWireVersion = 1;
constexpr std::uint8_t kBroadcastNode = 0;
constexpr std::uint32_t kSyncSamplesRequired = 6;
constexpr std::int64_t kMaxAcceptedRttUs = 50000;
constexpr std::int64_t kPlanLeadUs = 5000000;
constexpr std::int64_t kMinimumArmMarginUs = 500000;
constexpr std::int64_t kTimePlanLeadUs = 1500000;
constexpr std::int64_t kMinimumTimePlanMarginUs = 100000;
constexpr std::int64_t kNanoPiUartApplyCompensationUs = 4500;
constexpr double kFramePeriodUs = 1000000.0 / 30.0;
constexpr std::uint32_t kPulseWidthUs = 100;
constexpr gpio_num_t kStatusLedGpio = GPIO_NUM_18;
constexpr rmt_channel_t kStatusLedRmtChannel = RMT_CHANNEL_1;
constexpr std::uint8_t kStatusLedClockDivider = 8;  // 80 MHz APB -> 0.1 us ticks
constexpr std::uint32_t kStatusLedBlinkHalfPeriodMs = 400;

enum class StatusLedEffect : std::uint8_t {
    boot_dim_red = 0,
    blue_solid = 1,
    yellow_solid = 2,
    green_blink = 3,
    red_blink = 4,
};

enum class MessageType : std::uint8_t {
    ping = 1,
    pong = 2,
    plan = 3,
    arm = 4,
    stop = 5,
    abort_session = 6,
    status = 7,
    time_sync = 8,
};

enum class NodeState : std::uint32_t {
    boot = 0,
    idle = 1,
    waiting_neo_start = 2,
    ready = 3,
    armed = 4,
    running = 5,
    stopping = 6,
    draining = 7,
    fault = 8,
};

enum ErrorFlags : std::uint32_t {
    error_none = 0,
    error_bad_packet = 1U << 0,
    error_bad_plan = 1U << 1,
    error_neo_timeout = 1U << 2,
    error_rmt = 1U << 3,
    error_late_arm = 1U << 4,
    error_neo_rejected = 1U << 5,
    error_uart_tx_queue = 1U << 6,
    error_time_plan = 1U << 7,
};

#pragma pack(push, 1)
struct WireMessage {
    std::uint32_t magic;
    std::uint8_t version;
    std::uint8_t type;
    std::uint8_t node_id;
    std::uint8_t target_node;
    std::uint32_t sequence;
    std::uint32_t session_id;
    std::int64_t a;
    std::int64_t b;
    std::int64_t c;
    std::int64_t d;
    std::uint32_t x;
    std::uint32_t y;
    std::uint32_t crc32;
};
#pragma pack(pop)

static_assert(sizeof(WireMessage) == 60, "WireMessage layout changed");

std::uint32_t messageCrc(const WireMessage& message)
{
    return esp_crc32_le(UINT32_MAX,
                        reinterpret_cast<const std::uint8_t*>(&message),
                        offsetof(WireMessage, crc32));
}

void finishMessage(WireMessage& message)
{
    message.magic = kWireMagic;
    message.version = kWireVersion;
    message.crc32 = messageCrc(message);
}

bool validMessage(const WireMessage& message)
{
    return message.magic == kWireMagic &&
           message.version == kWireVersion &&
           message.crc32 == messageCrc(message);
}

const char* stateName(NodeState state)
{
    switch (state) {
    case NodeState::boot: return "BOOT";
    case NodeState::idle: return "IDLE";
    case NodeState::waiting_neo_start: return "WAIT_NEO";
    case NodeState::ready: return "READY";
    case NodeState::armed: return "ARMED";
    case NodeState::running: return "RUNNING";
    case NodeState::stopping: return "STOPPING";
    case NodeState::draining: return "DRAINING";
    case NodeState::fault: return "FAULT";
    }
    return "UNKNOWN";
}

String readUsbLine()
{
    static String line;
    while (Serial.available() > 0) {
        const char ch = static_cast<char>(Serial.read());
        if (ch == '\n') {
            String result = line;
            line = "";
            result.trim();
            return result;
        }
        if (ch != '\r' && line.length() < 160)
            line += ch;
    }
    return "";
}

std::uint32_t parseSession(const String& line, std::uint32_t fallback)
{
    const int separator = line.indexOf(' ');
    if (separator < 0)
        return fallback;
    const unsigned long value = line.substring(separator + 1).toInt();
    return value == 0 ? fallback : static_cast<std::uint32_t>(value);
}

bool gStatusLedReady = false;
StatusLedEffect gLastStatusLedEffect = StatusLedEffect::boot_dim_red;
bool gLastStatusLedOn = true;

void writeStatusLed(std::uint8_t red, std::uint8_t green, std::uint8_t blue)
{
    if (!gStatusLedReady)
        return;

    std::array<rmt_item32_t, 25> items{};
    const std::uint32_t grb =
        (static_cast<std::uint32_t>(green) << 16) |
        (static_cast<std::uint32_t>(red) << 8) |
        static_cast<std::uint32_t>(blue);
    for (std::size_t i = 0; i < 24; ++i) {
        const bool one = (grb & (UINT32_C(1) << (23 - i))) != 0;
        items[i].level0 = 1;
        items[i].duration0 = one ? 8 : 4;
        items[i].level1 = 0;
        items[i].duration1 = one ? 5 : 9;
    }
    items[24].level0 = 0;
    items[24].duration0 = 600;
    items[24].level1 = 0;
    items[24].duration1 = 1;
    rmt_write_items(kStatusLedRmtChannel, items.data(),
                    static_cast<int>(items.size()), true);
}

void initializeStatusLed()
{
    rmt_config_t config =
        RMT_DEFAULT_CONFIG_TX(kStatusLedGpio, kStatusLedRmtChannel);
    config.clk_div = kStatusLedClockDivider;
    config.mem_block_num = 1;
    config.tx_config.loop_en = false;
    config.tx_config.carrier_en = false;
    config.tx_config.idle_output_en = true;
    config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    if (rmt_config(&config) != ESP_OK ||
        rmt_driver_install(kStatusLedRmtChannel, 0, 0) != ESP_OK) {
        return;
    }
    gStatusLedReady = true;
    writeStatusLed(8, 0, 0);
}

void serviceStatusLed(StatusLedEffect effect)
{
    const bool blinking = effect == StatusLedEffect::green_blink ||
                          effect == StatusLedEffect::red_blink;
    const bool led_on = !blinking ||
        ((millis() / kStatusLedBlinkHalfPeriodMs) & 1U) == 0;
    if (effect == gLastStatusLedEffect && led_on == gLastStatusLedOn)
        return;

    gLastStatusLedEffect = effect;
    gLastStatusLedOn = led_on;
    if (!led_on) {
        writeStatusLed(0, 0, 0);
    } else if (effect == StatusLedEffect::boot_dim_red) {
        writeStatusLed(8, 0, 0);
    } else if (effect == StatusLedEffect::blue_solid) {
        writeStatusLed(0, 0, 48);
    } else if (effect == StatusLedEffect::yellow_solid) {
        writeStatusLed(48, 24, 0);
    } else if (effect == StatusLedEffect::green_blink) {
        writeStatusLed(0, 64, 0);
    } else {
        writeStatusLed(128, 0, 0);
    }
}

}  // namespace

#if defined(MODE2_ROLE_WEARABLE)

namespace {

static_assert(MODE2_NODE_ID >= 1 && MODE2_NODE_ID <= 3,
              "MODE2_NODE_ID must be 1, 2, or 3");

constexpr std::uint8_t kNodeId = MODE2_NODE_ID;
constexpr gpio_num_t kTriggerGpio = GPIO_NUM_2;
constexpr int kNeoRxPin = 41;
constexpr int kNeoTxPin = 42;
constexpr std::uint32_t kNeoBaud = 115200;
constexpr rmt_channel_t kRmtChannel = RMT_CHANNEL_0;
constexpr std::uint8_t kRmtClockDivider = 80;  // 80 MHz APB -> 1 us ticks
constexpr std::size_t kPatternPeriods = 16;
constexpr std::size_t kRmtItemCount = kPatternPeriods * 2;

struct TriggerPlan {
    std::uint32_t session_id = 0;
    std::uint32_t sequence = 0;
    std::int64_t local_epoch_us = 0;
    std::uint64_t local_period_q32_us = 0;
    std::uint32_t pulse_width_us = kPulseWidthUs;
};

struct TimePlan {
    std::uint32_t sequence = 0;
    std::int64_t local_epoch_us = 0;
    std::int64_t utc_epoch_ns = 0;
    std::uint32_t uncertainty_us = 0;
};

enum class NeoTxKind : std::uint8_t {
    text = 1,
    time_sync = 2,
};

struct NeoTxMessage {
    NeoTxKind kind = NeoTxKind::text;
    std::uint32_t sequence = 0;
    std::int64_t local_epoch_us = 0;
    std::int64_t utc_epoch_ns = 0;
    char text[96]{};
};

BLECharacteristic* gStatusCharacteristic = nullptr;
QueueHandle_t gBleTxQueue = nullptr;
QueueHandle_t gCommandQueue = nullptr;
QueueHandle_t gTimePlanQueue = nullptr;
QueueHandle_t gNeoControlTxQueue = nullptr;
QueueHandle_t gNeoTimeTxQueue = nullptr;
TaskHandle_t gTriggerTaskHandle = nullptr;
std::atomic<bool> gBleConnected{false};
std::atomic<NodeState> gNodeState{NodeState::boot};
std::atomic<std::uint32_t> gErrorFlags{error_none};
std::atomic<std::uint32_t> gSessionId{0};
std::atomic<std::uint64_t> gGeneratedPulses{0};
std::atomic<std::uint32_t> gTimeSyncCount{0};
std::atomic<std::uint32_t> gTimeSyncDrops{0};
std::atomic<std::uint32_t> gLastStoppedSession{0};
std::atomic<std::int64_t> gLastStopFrames{-1};
std::atomic<bool> gLedEverConnected{false};
std::atomic<bool> gLedTimeReady{false};
std::atomic<bool> gLedCollecting{false};

portMUX_TYPE gPlanMux = portMUX_INITIALIZER_UNLOCKED;
TriggerPlan gPlan;
std::int64_t gStopEpochUs = 0;
rmt_item32_t gRmtItems[kRmtItemCount]{};
std::size_t gRmtItemsUsed = 0;
std::int64_t gRunStartUs = 0;
std::int64_t gNeoDeadlineUs = 0;
String gNeoLine;

void notifyNode(const WireMessage& source)
{
    if (!gBleConnected.load() || gBleTxQueue == nullptr)
        return;

    WireMessage message = source;
    xQueueSend(gBleTxQueue, &message, 0);
}

void bleNotifyTask(void*)
{
    WireMessage message{};
    for (;;) {
        if (xQueueReceive(gBleTxQueue, &message, portMAX_DELAY) != pdTRUE)
            continue;
        if (!gBleConnected.load() || gStatusCharacteristic == nullptr)
            continue;
        if (static_cast<MessageType>(message.type) == MessageType::pong)
            message.c = esp_timer_get_time();
        finishMessage(message);
        gStatusCharacteristic->setValue(
            reinterpret_cast<std::uint8_t*>(&message), sizeof(message));
        gStatusCharacteristic->notify();
    }
}

void sendStatus()
{
    const std::int64_t now_us = esp_timer_get_time();
    if (gNodeState.load() == NodeState::running && gRunStartUs > 0 &&
        now_us >= gRunStartUs) {
        gGeneratedPulses.store(static_cast<std::uint64_t>(
            std::floor((now_us - gRunStartUs) / kFramePeriodUs) + 1.0));
    }
    WireMessage message{};
    message.type = static_cast<std::uint8_t>(MessageType::status);
    message.node_id = kNodeId;
    message.target_node = kBroadcastNode;
    message.session_id = gSessionId.load();
    message.a = now_us;
    message.b = gRunStartUs;
    message.c = static_cast<std::int64_t>(gGeneratedPulses.load());
    message.d = gLastStopFrames.load();
    message.sequence = gLastStoppedSession.load();
    message.x = static_cast<std::uint32_t>(gNodeState.load());
    message.y = gErrorFlags.load();
    notifyNode(message);
}

bool parseUnsignedField(const String& line, const char* field,
                        std::uint64_t& value)
{
    const int start = line.indexOf(field);
    if (start < 0)
        return false;

    const int value_start = start + static_cast<int>(std::strlen(field));
    int value_end = line.indexOf('+', value_start);
    if (value_end < 0)
        value_end = line.length();
    const String text = line.substring(value_start, value_end);
    if (text.length() == 0)
        return false;

    char* end = nullptr;
    errno = 0;
    const unsigned long long parsed = std::strtoull(text.c_str(), &end, 10);
    if (errno != 0 || end == text.c_str() || *end != '\0' ||
        parsed > static_cast<unsigned long long>(INT64_MAX)) {
        return false;
    }
    value = static_cast<std::uint64_t>(parsed);
    return true;
}

bool enqueueNeoControl(const char* verb, std::uint32_t session_id)
{
    NeoTxMessage message{};
    message.kind = NeoTxKind::text;
    const int length = std::snprintf(
        message.text, sizeof(message.text),
        "CMD+%s+SESSION=%lu+END\r\n", verb,
        static_cast<unsigned long>(session_id));
    if (length <= 0 || static_cast<std::size_t>(length) >= sizeof(message.text) ||
        gNeoControlTxQueue == nullptr ||
        xQueueSend(gNeoControlTxQueue, &message, 0) != pdTRUE) {
        gErrorFlags.fetch_or(error_uart_tx_queue);
        return false;
    }
    return true;
}

bool formatTimeSyncLine(std::int64_t utc_ns, char* output, std::size_t capacity)
{
    if (utc_ns <= 0 || output == nullptr || capacity == 0)
        return false;

    const std::int64_t seconds = utc_ns / 1000000000LL;
    const std::int64_t microseconds = (utc_ns % 1000000000LL) / 1000LL;
    const std::time_t utc_seconds = static_cast<std::time_t>(seconds);
    std::tm utc{};
    if (gmtime_r(&utc_seconds, &utc) == nullptr)
        return false;

    const int length = std::snprintf(
        output, capacity,
        "TIMESYNC+%04d-%02d-%02dT%02d:%02d:%02d.%06lldZ+END\r\n",
        utc.tm_year + 1900, utc.tm_mon + 1, utc.tm_mday,
        utc.tm_hour, utc.tm_min, utc.tm_sec,
        static_cast<long long>(microseconds));
    return length > 0 && static_cast<std::size_t>(length) < capacity;
}

void neoUartWriterTask(void*)
{
    NeoTxMessage message{};
    for (;;) {
        bool have_message =
            xQueueReceive(gNeoControlTxQueue, &message, 0) == pdTRUE;
        if (!have_message) {
            have_message = xQueueReceive(
                gNeoTimeTxQueue, &message, pdMS_TO_TICKS(2)) == pdTRUE;
        }
        if (!have_message)
            continue;

        char time_line[96]{};
        const char* data = message.text;
        if (message.kind == NeoTxKind::time_sync) {
            const std::int64_t now_us = esp_timer_get_time();
            const std::int64_t utc_at_line_receive_ns =
                message.utc_epoch_ns +
                (now_us - message.local_epoch_us +
                 kNanoPiUartApplyCompensationUs) * 1000LL;
            if (!formatTimeSyncLine(
                    utc_at_line_receive_ns, time_line, sizeof(time_line))) {
                gTimeSyncDrops.fetch_add(1);
                continue;
            }
            data = time_line;
        }

        const std::size_t length = std::strlen(data);
        const std::size_t written = Serial1.write(
            reinterpret_cast<const std::uint8_t*>(data), length);
        if (written != length) {
            gErrorFlags.fetch_or(error_uart_tx_queue);
            if (message.kind == NeoTxKind::time_sync)
                gTimeSyncDrops.fetch_add(1);
            continue;
        }
        if (message.kind == NeoTxKind::time_sync) {
            gTimeSyncCount.fetch_add(1);
            gLedTimeReady.store(true);
            Serial.printf("[TIME] seq=%lu queued to NanoPi UART\n",
                          static_cast<unsigned long>(message.sequence));
        }
    }
}

void timeSyncTask(void*)
{
    WireMessage wire{};
    for (;;) {
        if (xQueueReceive(gTimePlanQueue, &wire, portMAX_DELAY) != pdTRUE)
            continue;

        TimePlan plan{};
        plan.sequence = wire.sequence;
        plan.local_epoch_us = wire.a;
        plan.utc_epoch_ns = wire.b;
        plan.uncertainty_us = wire.x;

        std::int64_t remaining = plan.local_epoch_us - esp_timer_get_time();
        if (remaining < kMinimumTimePlanMarginUs || plan.utc_epoch_ns <= 0) {
            gErrorFlags.fetch_or(error_time_plan);
            gTimeSyncDrops.fetch_add(1);
            continue;
        }

        while ((remaining = plan.local_epoch_us - esp_timer_get_time()) > 0) {
            if (remaining > 10000) {
                const std::uint32_t delay_ms = static_cast<std::uint32_t>(
                    std::min<std::int64_t>(remaining / 1000 - 3, 20));
                vTaskDelay(pdMS_TO_TICKS(delay_ms == 0 ? 1 : delay_ms));
            } else {
                taskYIELD();
            }
        }

        NeoTxMessage tx{};
        tx.kind = NeoTxKind::time_sync;
        tx.sequence = plan.sequence;
        tx.local_epoch_us = plan.local_epoch_us;
        tx.utc_epoch_ns = plan.utc_epoch_ns;
        xQueueOverwrite(gNeoTimeTxQueue, &tx);
    }
}

void stopRmtNow()
{
    rmt_tx_stop(kRmtChannel);
    gpio_set_level(kTriggerGpio, 0);
}

bool buildRmtPattern(const TriggerPlan& plan)
{
    if (plan.pulse_width_us < 50 || plan.pulse_width_us > 1000)
        return false;

    std::uint64_t cumulative_q32 = 0;
    std::uint64_t previous_rounded_us = 0;
    std::size_t item_index = 0;

    for (std::size_t i = 0; i < kPatternPeriods; ++i) {
        cumulative_q32 += plan.local_period_q32_us;
        const std::uint64_t rounded_us =
            (cumulative_q32 + (UINT64_C(1) << 31)) >> 32;
        const std::uint32_t period_us = static_cast<std::uint32_t>(
            rounded_us - previous_rounded_us);
        previous_rounded_us = rounded_us;

        if (period_us <= plan.pulse_width_us + 2 || period_us > 60000)
            return false;

        const std::uint32_t low_us = period_us - plan.pulse_width_us;
        const std::uint32_t low_first = low_us / 2;
        const std::uint32_t low_second = low_us - low_first;
        if (low_first == 0 || low_second == 0 ||
            low_first > 32767 || low_second > 32767)
            return false;

        rmt_item32_t& first = gRmtItems[item_index++];
        first.level0 = 1;
        first.duration0 = plan.pulse_width_us;
        first.level1 = 0;
        first.duration1 = low_first;

        rmt_item32_t& second = gRmtItems[item_index++];
        second.level0 = 0;
        second.duration0 = low_second;
        second.level1 = 0;
        second.duration1 = 1;

        // The second low phase adds one tick; remove it from its first phase.
        --second.duration0;
    }

    gRmtItemsUsed = item_index;
    return item_index == kRmtItemCount;
}

bool initializeRmt()
{
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(kTriggerGpio, kRmtChannel);
    config.clk_div = kRmtClockDivider;
    config.mem_block_num = 1;
    config.tx_config.loop_en = true;
    config.tx_config.carrier_en = false;
    config.tx_config.idle_output_en = true;
    config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;

    if (rmt_config(&config) != ESP_OK)
        return false;
    if (rmt_driver_install(kRmtChannel, 0, ESP_INTR_FLAG_IRAM) != ESP_OK)
        return false;
    if (rmt_set_tx_loop_mode(kRmtChannel, true) != ESP_OK)
        return false;
    gpio_set_level(kTriggerGpio, 0);
    return true;
}

TriggerPlan copyPlan()
{
    TriggerPlan copy;
    portENTER_CRITICAL(&gPlanMux);
    copy = gPlan;
    portEXIT_CRITICAL(&gPlanMux);
    return copy;
}

std::int64_t copyStopEpoch()
{
    portENTER_CRITICAL(&gPlanMux);
    const std::int64_t copy = gStopEpochUs;
    portEXIT_CRITICAL(&gPlanMux);
    return copy;
}

bool waitUntil(std::int64_t epoch_us, NodeState required_state)
{
    while (gNodeState.load() == required_state) {
        const std::int64_t remaining = epoch_us - esp_timer_get_time();
        if (remaining <= 0)
            return true;
        if (remaining > 10000)
            vTaskDelay(pdMS_TO_TICKS(static_cast<std::uint32_t>(remaining / 1000 - 3)));
        else if (remaining > 2000)
            taskYIELD();
        else {
            while (esp_timer_get_time() < epoch_us) {
                if (gNodeState.load() != required_state)
                    return false;
            }
            return true;
        }
    }
    return false;
}

void triggerTask(void*)
{
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        NodeState state = gNodeState.load();

        if (state == NodeState::armed) {
            const TriggerPlan plan = copyPlan();
            if (!waitUntil(plan.local_epoch_us, NodeState::armed))
                continue;

            const std::int64_t lateness = esp_timer_get_time() - plan.local_epoch_us;
            if (lateness > 2000) {
                gErrorFlags.fetch_or(error_late_arm);
                gNodeState.store(NodeState::fault);
                gpio_set_level(kTriggerGpio, 0);
                sendStatus();
                continue;
            }

            if (rmt_write_items(kRmtChannel, gRmtItems,
                                static_cast<int>(gRmtItemsUsed), false) != ESP_OK) {
                gErrorFlags.fetch_or(error_rmt);
                gNodeState.store(NodeState::fault);
                sendStatus();
                continue;
            }
            gRunStartUs = plan.local_epoch_us;
            gGeneratedPulses.store(0);
            gLedCollecting.store(true);
            gNodeState.store(NodeState::running);
            sendStatus();
        }

        state = gNodeState.load();
        if (state == NodeState::stopping) {
            const std::int64_t stop_epoch = copyStopEpoch();
            if (stop_epoch > esp_timer_get_time())
                waitUntil(stop_epoch, NodeState::stopping);
            if (gNodeState.load() != NodeState::stopping)
                continue;

            stopRmtNow();
            gLedCollecting.store(false);
            const TriggerPlan plan = copyPlan();
            if (gRunStartUs > 0 && stop_epoch > gRunStartUs) {
                const double period =
                    static_cast<double>(plan.local_period_q32_us) /
                    4294967296.0;
                gGeneratedPulses.store(static_cast<std::uint64_t>(
                    std::floor((stop_epoch - gRunStartUs) / period) + 1.0));
            }
            gNodeState.store(NodeState::draining);
            if (!enqueueNeoControl("STOP", plan.session_id)) {
                gNodeState.store(NodeState::fault);
                sendStatus();
                continue;
            }
            gNeoDeadlineUs = esp_timer_get_time() + 15000000;
            sendStatus();
        }
    }
}

class NodeServerCallbacks final : public BLEServerCallbacks {
public:
    void onConnect(BLEServer*) override
    {
        gBleConnected.store(true);
        gLedEverConnected.store(true);
    }

    void onDisconnect(BLEServer*) override
    {
        gBleConnected.store(false);
        gLedTimeReady.store(false);
        if (gBleTxQueue != nullptr)
            xQueueReset(gBleTxQueue);
        BLEDevice::startAdvertising();
    }
};

class NodeCommandCallbacks final : public BLECharacteristicCallbacks {
public:
    void onWrite(BLECharacteristic* characteristic) override
    {
        const std::string value = characteristic->getValue();
        if (value.size() != sizeof(WireMessage)) {
            gErrorFlags.fetch_or(error_bad_packet);
            return;
        }

        WireMessage message{};
        std::memcpy(&message, value.data(), sizeof(message));
        if (!validMessage(message) ||
            (message.target_node != kBroadcastNode &&
             message.target_node != kNodeId)) {
            gErrorFlags.fetch_or(error_bad_packet);
            return;
        }

        const MessageType type = static_cast<MessageType>(message.type);
        if (type == MessageType::ping) {
            const std::int64_t receive_us = esp_timer_get_time();
            WireMessage pong{};
            pong.type = static_cast<std::uint8_t>(MessageType::pong);
            pong.node_id = kNodeId;
            pong.target_node = kBroadcastNode;
            pong.sequence = message.sequence;
            pong.a = message.a;
            pong.b = receive_us;
            pong.c = esp_timer_get_time();
            notifyNode(pong);
            return;
        }

        if (type == MessageType::time_sync) {
            if (gTimePlanQueue == nullptr ||
                xQueueOverwrite(gTimePlanQueue, &message) != pdTRUE) {
                gErrorFlags.fetch_or(error_time_plan);
                gTimeSyncDrops.fetch_add(1);
            }
            return;
        }

        if (xQueueSend(gCommandQueue, &message, 0) != pdTRUE)
            gErrorFlags.fetch_or(error_bad_packet);
    }
};

NodeServerCallbacks gServerCallbacks;
NodeCommandCallbacks gCommandCallbacks;

void initializeBleNode()
{
    char name[24];
    std::snprintf(name, sizeof(name), "Mode2Node-%u", kNodeId);
    BLEDevice::init(name);
    BLEDevice::setMTU(185);
    BLEDevice::setPower(ESP_PWR_LVL_P9);

    BLEServer* server = BLEDevice::createServer();
    server->setCallbacks(&gServerCallbacks);
    BLEService* service = server->createService(kServiceUuid);

    BLECharacteristic* command = service->createCharacteristic(
        kCommandUuid, BLECharacteristic::PROPERTY_WRITE |
                          BLECharacteristic::PROPERTY_WRITE_NR);
    command->setCallbacks(&gCommandCallbacks);

    gStatusCharacteristic = service->createCharacteristic(
        kStatusUuid,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_NOTIFY);
    gStatusCharacteristic->addDescriptor(new BLE2902());
    service->start();

    std::uint8_t manufacturer[2] = {0xD2, kNodeId};
    BLEAdvertising* advertising = BLEDevice::getAdvertising();
    BLEAdvertisementData advertisement_data;
    advertisement_data.setFlags(0x06);
    advertisement_data.setCompleteServices(BLEUUID(kServiceUuid));
    advertisement_data.setManufacturerData(std::string(
        reinterpret_cast<char*>(manufacturer), sizeof(manufacturer)));
    advertising->setAdvertisementData(advertisement_data);
    BLEAdvertisementData scan_response;
    scan_response.setName(name);
    advertising->setScanResponseData(scan_response);
    advertising->setMinPreferred(0x06);
    BLEDevice::startAdvertising();
}

void processNodeCommand(const WireMessage& message)
{
    const MessageType type = static_cast<MessageType>(message.type);

    if (type == MessageType::plan) {
        const NodeState state = gNodeState.load();
        if (state != NodeState::idle && state != NodeState::fault) {
            gErrorFlags.fetch_or(error_bad_plan);
            sendStatus();
            return;
        }

        TriggerPlan plan;
        plan.session_id = message.session_id;
        plan.sequence = message.sequence;
        plan.local_epoch_us = message.a;
        plan.local_period_q32_us = static_cast<std::uint64_t>(message.b);
        plan.pulse_width_us = message.x;

        const double period_us =
            static_cast<double>(plan.local_period_q32_us) / 4294967296.0;
        if (plan.local_epoch_us - esp_timer_get_time() < kMinimumArmMarginUs ||
            period_us < 30000.0 || period_us > 37000.0 ||
            !buildRmtPattern(plan)) {
            gErrorFlags.fetch_or(error_bad_plan);
            gNodeState.store(NodeState::fault);
            sendStatus();
            return;
        }

        stopRmtNow();
        portENTER_CRITICAL(&gPlanMux);
        gPlan = plan;
        gStopEpochUs = 0;
        portEXIT_CRITICAL(&gPlanMux);
        gSessionId.store(plan.session_id);
        gErrorFlags.store(error_none);
        gRunStartUs = 0;
        gGeneratedPulses.store(0);
        gNodeState.store(NodeState::waiting_neo_start);
        if (!enqueueNeoControl("START", plan.session_id)) {
            gNodeState.store(NodeState::fault);
            sendStatus();
            return;
        }
        gNeoDeadlineUs = esp_timer_get_time() + 2500000;
        sendStatus();
        return;
    }

    if (type == MessageType::arm) {
        if (message.session_id == gSessionId.load() &&
            gNodeState.load() == NodeState::ready) {
            const TriggerPlan plan = copyPlan();
            if (plan.local_epoch_us - esp_timer_get_time() < 200000) {
                gErrorFlags.fetch_or(error_late_arm);
                gNodeState.store(NodeState::fault);
            } else {
                gNodeState.store(NodeState::armed);
                xTaskNotifyGive(gTriggerTaskHandle);
            }
            sendStatus();
        }
        return;
    }

    if (type == MessageType::stop) {
        if (message.session_id != gSessionId.load())
            return;
        portENTER_CRITICAL(&gPlanMux);
        gStopEpochUs = message.a;
        portEXIT_CRITICAL(&gPlanMux);
        gNodeState.store(NodeState::stopping);
        xTaskNotifyGive(gTriggerTaskHandle);
        sendStatus();
        return;
    }

    if (type == MessageType::abort_session) {
        stopRmtNow();
        gLedCollecting.store(false);
        const std::uint32_t session = gSessionId.load();
        if (session != 0 && !enqueueNeoControl("STOP", session)) {
            gNodeState.store(NodeState::fault);
            sendStatus();
            return;
        }
        gNodeState.store(NodeState::idle);
        gSessionId.store(0);
        gRunStartUs = 0;
        sendStatus();
    }
}

void pollNeoUart()
{
    while (Serial1.available() > 0) {
        const char ch = static_cast<char>(Serial1.read());
        if (ch == '\n') {
            String line = gNeoLine;
            gNeoLine = "";
            line.trim();

            if (line.startsWith("ACK+START+") &&
                line.indexOf("+OK+") >= 0 &&
                gNodeState.load() == NodeState::waiting_neo_start) {
                gNeoDeadlineUs = 0;
                gNodeState.store(NodeState::ready);
                sendStatus();
            } else if (line.startsWith("ACK+START+") &&
                       line.indexOf("+ERROR+") >= 0 &&
                       gNodeState.load() == NodeState::waiting_neo_start) {
                gNeoDeadlineUs = 0;
                gErrorFlags.fetch_or(error_neo_rejected);
                gNodeState.store(NodeState::fault);
                sendStatus();
            } else if (line.startsWith("ACK+STOP+") &&
                       line.indexOf("+OK+") >= 0 &&
                       gNodeState.load() == NodeState::draining) {
                const std::uint32_t stopped_session = gSessionId.load();
                std::uint64_t frames = 0;
                gLastStoppedSession.store(stopped_session);
                gLastStopFrames.store(parseUnsignedField(line, "+FRAMES=", frames)
                    ? static_cast<std::int64_t>(frames) : -1);
                gNeoDeadlineUs = 0;
                gNodeState.store(NodeState::idle);
                gSessionId.store(0);
                gRunStartUs = 0;
                sendStatus();
            } else if (line.startsWith("ACK+STOP+") &&
                       line.indexOf("+ERROR+") >= 0 &&
                       gNodeState.load() == NodeState::draining) {
                gNeoDeadlineUs = 0;
                gErrorFlags.fetch_or(error_neo_rejected);
                gNodeState.store(NodeState::fault);
                sendStatus();
            }
            Serial.printf("[NEO] %s\n", line.c_str());
        } else if (ch != '\r' && gNeoLine.length() < 240) {
            gNeoLine += ch;
        }
    }

    const NodeState state = gNodeState.load();
    if ((state == NodeState::waiting_neo_start || state == NodeState::draining) &&
        gNeoDeadlineUs > 0 && esp_timer_get_time() > gNeoDeadlineUs) {
        gErrorFlags.fetch_or(error_neo_timeout);
        stopRmtNow();
        gNodeState.store(NodeState::fault);
        gNeoDeadlineUs = 0;
        sendStatus();
    }
}

void printNodeStatus()
{
    const TriggerPlan plan = copyPlan();
    Serial.printf("node=%u state=%s session=%lu error=0x%08lx "
                  "epoch=%lld period=%.6fus pulses=%llu ble=%s "
                  "time_sent=%lu time_drop=%lu\n",
                  kNodeId, stateName(gNodeState.load()),
                  static_cast<unsigned long>(gSessionId.load()),
                  static_cast<unsigned long>(gErrorFlags.load()),
                  static_cast<long long>(plan.local_epoch_us),
                  static_cast<double>(plan.local_period_q32_us) / 4294967296.0,
                  static_cast<unsigned long long>(gGeneratedPulses.load()),
                  gBleConnected.load() ? "connected" : "disconnected",
                  static_cast<unsigned long>(gTimeSyncCount.load()),
                  static_cast<unsigned long>(gTimeSyncDrops.load()));
}

StatusLedEffect wearableStatusLedEffect()
{
    const NodeState state = gNodeState.load();
    const bool connected = gBleConnected.load();
    const bool abnormal = gErrorFlags.load() != error_none ||
                          state == NodeState::fault || !connected;
    if (gLedCollecting.load()) {
        return abnormal ? StatusLedEffect::red_blink
                        : StatusLedEffect::green_blink;
    }
    if ((gLedEverConnected.load() && !connected) ||
        gErrorFlags.load() != error_none || state == NodeState::fault) {
        return StatusLedEffect::yellow_solid;
    }
    if (connected && gLedTimeReady.load() &&
        (state == NodeState::idle ||
         state == NodeState::waiting_neo_start ||
         state == NodeState::ready || state == NodeState::armed)) {
        return StatusLedEffect::blue_solid;
    }
    return StatusLedEffect::boot_dim_red;
}

void wearableStatusLedTask(void*)
{
    for (;;) {
        serviceStatusLed(wearableStatusLedEffect());
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

}  // namespace

void setup()
{
    Serial.begin(115200);
    delay(300);
    Serial1.begin(kNeoBaud, SERIAL_8N1, kNeoRxPin, kNeoTxPin);
    initializeStatusLed();
    xTaskCreatePinnedToCore(wearableStatusLedTask, "status_led", 2048,
                            nullptr, 2, nullptr, 0);

    gBleTxQueue = xQueueCreate(24, sizeof(WireMessage));
    gCommandQueue = xQueueCreate(12, sizeof(WireMessage));
    gTimePlanQueue = xQueueCreate(1, sizeof(WireMessage));
    gNeoControlTxQueue = xQueueCreate(8, sizeof(NeoTxMessage));
    gNeoTimeTxQueue = xQueueCreate(1, sizeof(NeoTxMessage));
    if (gBleTxQueue == nullptr || gCommandQueue == nullptr ||
        gTimePlanQueue == nullptr || gNeoControlTxQueue == nullptr ||
        gNeoTimeTxQueue == nullptr || !initializeRmt()) {
        Serial.println("FATAL: queue/mutex/RMT initialization failed");
        gErrorFlags.store(error_rmt);
        gNodeState.store(NodeState::fault);
        return;
    }

    xTaskCreatePinnedToCore(triggerTask, "camera_trigger", 4096, nullptr, 10,
                            &gTriggerTaskHandle, 1);
    xTaskCreatePinnedToCore(neoUartWriterTask, "neo_uart_tx", 4096,
                            nullptr, 6, nullptr, 0);
    xTaskCreatePinnedToCore(timeSyncTask, "time_sync", 4096,
                            nullptr, 3, nullptr, 0);
    initializeBleNode();
    xTaskCreatePinnedToCore(bleNotifyTask, "ble_notify", 4096,
                            nullptr, 5, nullptr, 0);
    gNodeState.store(NodeState::idle);
    Serial.printf("Mode2+Time wearable node %u ready; trigger=GPIO%d, Neo UART RX=%d TX=%d\n",
                  kNodeId, static_cast<int>(kTriggerGpio), kNeoRxPin, kNeoTxPin);
}

void loop()
{
    WireMessage message{};
    while (gCommandQueue != nullptr && xQueueReceive(gCommandQueue, &message, 0) == pdTRUE)
        processNodeCommand(message);

    pollNeoUart();

    const String command = readUsbLine();
    if (command.equalsIgnoreCase("STATUS"))
        printNodeStatus();
    else if (command.equalsIgnoreCase("ABORT")) {
        WireMessage abort{};
        abort.type = static_cast<std::uint8_t>(MessageType::abort_session);
        abort.session_id = gSessionId.load();
        processNodeCommand(abort);
    }

    static std::uint32_t last_status_ms = 0;
    if (millis() - last_status_ms >= 500) {
        last_status_ms = millis();
        sendStatus();
    }
    delay(2);
}

#endif  // MODE2_ROLE_WEARABLE

#if defined(MODE2_ROLE_COORDINATOR)

namespace {

static_assert(MODE2_NODE_COUNT >= 1 && MODE2_NODE_COUNT <= 3,
              "MODE2_NODE_COUNT must be 1, 2, or 3");
constexpr std::size_t kNodeCount = MODE2_NODE_COUNT;

struct LinkState {
    LinkState() = default;
    explicit LinkState(std::uint8_t id) : node_id(id) {}

    std::uint8_t node_id = 0;
    std::string address;
    esp_ble_addr_type_t address_type = BLE_ADDR_TYPE_PUBLIC;
    BLEClient* client = nullptr;
    BLERemoteCharacteristic* command = nullptr;
    bool connected = false;
    NodeState node_state = NodeState::boot;
    std::uint32_t node_session = 0;
    std::uint32_t node_errors = 0;
    std::int64_t last_status_us = 0;
    std::uint32_t last_stop_session = 0;
    std::int64_t last_stop_frames = -1;

    std::uint32_t sync_samples = 0;
    std::int64_t ref_coordinator_us = 0;
    double ref_local_us = 0.0;
    double local_rate = 1.0;
    std::int64_t last_sample_coordinator_us = 0;
    double last_sample_local_us = 0.0;
    std::int64_t last_rtt_us = INT64_MAX;
};

struct UtcClockMap {
    bool valid = false;
    std::uint32_t sequence = 0;
    std::int64_t coordinator_ref_us = 0;
    std::int64_t utc_ref_ns = 0;
    std::uint32_t uncertainty_us = 0;
};

struct PendingTimeDistribution {
    bool active = false;
    std::uint32_t sequence = 0;
    std::int64_t coordinator_epoch_us = 0;
    std::int64_t utc_epoch_ns = 0;
    std::uint32_t uncertainty_us = 0;
    std::size_t next_link = 0;
    std::size_t sent_links = 0;
};

struct DiscoveredNode {
    std::uint8_t node_id = 0;
    esp_ble_addr_type_t address_type = BLE_ADDR_TYPE_PUBLIC;
    char address[18]{};
};

std::array<LinkState, kNodeCount> makeLinks()
{
    std::array<LinkState, kNodeCount> links{};
    for (std::size_t i = 0; i < links.size(); ++i)
        links[i].node_id = static_cast<std::uint8_t>(i + 1);
    return links;
}

std::array<LinkState, kNodeCount> gLinks = makeLinks();
portMUX_TYPE gLinksMux = portMUX_INITIALIZER_UNLOCKED;
std::uint32_t gSequence = 0;
std::int64_t gLastPingUs = 0;
std::size_t gNextPingLink = 0;

bool gStartPending = false;
bool gArmSent = false;
std::uint32_t gPendingSession = 0;
std::int64_t gPendingCoordinatorEpochUs = 0;
UtcClockMap gUtcClockMap;
PendingTimeDistribution gPendingTimeDistribution;
std::uint32_t gFallbackTimeSequence = 0;
std::atomic<bool> gLedSessionActive{false};
std::atomic<std::uint8_t> gLedParticipantMask{0};
QueueHandle_t gDiscoveredNodeQueue = nullptr;
std::atomic<bool> gScanInProgress{false};
std::atomic<std::uint32_t> gPendingDiscoveredNodes{0};
std::atomic<std::int64_t> gBleWriteStartedUs{0};
std::atomic<std::uint32_t> gBleWriteSequence{0};
std::atomic<std::uint8_t> gBleWriteType{0};
std::atomic<std::uint8_t> gBleWriteTarget{0};

LinkState* linkForNode(std::uint8_t node_id)
{
    if (node_id < 1 || node_id > kNodeCount)
        return nullptr;
    return &gLinks[node_id - 1];
}

void handleNotification(BLERemoteCharacteristic*, std::uint8_t* data,
                        std::size_t length, bool)
{
    if (length != sizeof(WireMessage))
        return;
    WireMessage message{};
    std::memcpy(&message, data, sizeof(message));
    if (!validMessage(message))
        return;

    LinkState* link = linkForNode(message.node_id);
    if (link == nullptr)
        return;

    const MessageType type = static_cast<MessageType>(message.type);
    const std::int64_t now_us = esp_timer_get_time();
    bool report_stop_frames = false;
    std::uint32_t stop_session = 0;
    std::int64_t stop_frames = -1;

    portENTER_CRITICAL(&gLinksMux);
    if (type == MessageType::pong) {
        const std::int64_t t1 = message.a;
        const std::int64_t t2 = message.b;
        const std::int64_t t3 = message.c;
        const std::int64_t t4 = now_us;
        const std::int64_t rtt = (t4 - t1) - (t3 - t2);
        if (rtt >= 0 && rtt <= kMaxAcceptedRttUs) {
            const std::int64_t coordinator_mid = t1 + (t4 - t1) / 2;
            const double local_mid =
                static_cast<double>(t2) + static_cast<double>(t3 - t2) / 2.0;

            if (link->last_sample_coordinator_us == 0) {
                link->last_sample_coordinator_us = coordinator_mid;
                link->last_sample_local_us = local_mid;
            } else {
                const std::int64_t delta_coordinator =
                    coordinator_mid - link->last_sample_coordinator_us;
                const double delta_local = local_mid - link->last_sample_local_us;
                if (delta_coordinator >= 2000000) {
                    const double measured_rate =
                        delta_local / static_cast<double>(delta_coordinator);
                    if (measured_rate > 0.9995 && measured_rate < 1.0005)
                        link->local_rate =
                            0.85 * link->local_rate + 0.15 * measured_rate;
                    link->last_sample_coordinator_us = coordinator_mid;
                    link->last_sample_local_us = local_mid;
                }
            }
            link->ref_coordinator_us = coordinator_mid;
            link->ref_local_us = local_mid;
            link->last_rtt_us = rtt;
            ++link->sync_samples;
        }
    } else if (type == MessageType::status) {
        link->node_state = static_cast<NodeState>(message.x);
        link->node_session = message.session_id;
        link->node_errors = message.y;
        link->last_status_us = now_us;
        if (message.sequence != 0 &&
            (message.sequence != link->last_stop_session ||
             message.d != link->last_stop_frames)) {
            link->last_stop_session = message.sequence;
            link->last_stop_frames = message.d;
            report_stop_frames = true;
            stop_session = message.sequence;
            stop_frames = message.d;
        }
    }
    portEXIT_CRITICAL(&gLinksMux);

    if (report_stop_frames) {
        Serial.printf("STOP_FRAME session=%lu node=%u frames=%lld\n",
                      static_cast<unsigned long>(stop_session),
                      static_cast<unsigned int>(message.node_id),
                      static_cast<long long>(stop_frames));
    }
}

class CoordinatorClientCallbacks final : public BLEClientCallbacks {
public:
    void onConnect(BLEClient*) override {}

    void onDisconnect(BLEClient* client) override
    {
        portENTER_CRITICAL(&gLinksMux);
        for (auto& link : gLinks) {
            if (link.client == client) {
                link.connected = false;
                link.command = nullptr;
                link.sync_samples = 0;
                link.node_state = NodeState::boot;
            }
        }
        portEXIT_CRITICAL(&gLinksMux);
    }
};

CoordinatorClientCallbacks gClientCallbacks;

bool sendMessage(LinkState& link, WireMessage message, bool response = true)
{
    if (!link.connected || link.client == nullptr ||
        !link.client->isConnected() || link.command == nullptr)
        return false;

    message.node_id = kBroadcastNode;
    message.target_node = link.node_id;
    finishMessage(message);
    const std::int64_t write_started_us = esp_timer_get_time();
    gBleWriteSequence.store(message.sequence);
    gBleWriteType.store(message.type);
    gBleWriteTarget.store(link.node_id);
    gBleWriteStartedUs.store(write_started_us);
    link.command->writeValue(reinterpret_cast<std::uint8_t*>(&message),
                             sizeof(message), response);
    const std::int64_t elapsed_us = esp_timer_get_time() - write_started_us;
    gBleWriteStartedUs.store(0);
    if (elapsed_us > 100000) {
        Serial.printf("BLE_WRITE_SLOW node=%u type=%u seq=%lu response=%d "
                      "elapsed_us=%lld connected=%d\n",
                      static_cast<unsigned>(link.node_id),
                      static_cast<unsigned>(message.type),
                      static_cast<unsigned long>(message.sequence), response,
                      static_cast<long long>(elapsed_us),
                      link.client != nullptr && link.client->isConnected());
    }
    return link.client != nullptr && link.client->isConnected();
}

void coordinatorGattcEventHandler(esp_gattc_cb_event_t event,
                                  esp_gatt_if_t,
                                  esp_ble_gattc_cb_param_t* parameters)
{
    if (event != ESP_GATTC_DISCONNECT_EVT || parameters == nullptr)
        return;

    const std::int64_t now_us = esp_timer_get_time();
    const std::int64_t write_started_us = gBleWriteStartedUs.load();
    const std::int64_t pending_us = write_started_us > 0
        ? now_us - write_started_us : 0;
    const std::uint8_t* address = parameters->disconnect.remote_bda;
    Serial.printf("BLE_DISCONNECT reason=0x%02x conn_id=%u "
                  "addr=%02x:%02x:%02x:%02x:%02x:%02x "
                  "pending_node=%u pending_type=%u pending_seq=%lu "
                  "pending_us=%lld\n",
                  static_cast<unsigned>(parameters->disconnect.reason),
                  static_cast<unsigned>(parameters->disconnect.conn_id),
                  address[0], address[1], address[2],
                  address[3], address[4], address[5],
                  static_cast<unsigned>(gBleWriteTarget.load()),
                  static_cast<unsigned>(gBleWriteType.load()),
                  static_cast<unsigned long>(gBleWriteSequence.load()),
                  static_cast<long long>(pending_us));
}

bool connectLink(LinkState& link)
{
    if (link.address.empty())
        return false;
    if (link.client == nullptr) {
        link.client = BLEDevice::createClient();
        link.client->setClientCallbacks(&gClientCallbacks);
    }

    if (!link.client->isConnected() &&
        !link.client->connect(BLEAddress(link.address), link.address_type))
        return false;

    link.client->setMTU(185);
    BLERemoteService* service = link.client->getService(kServiceUuid);
    if (service == nullptr)
        return false;
    BLERemoteCharacteristic* command = service->getCharacteristic(kCommandUuid);
    BLERemoteCharacteristic* status = service->getCharacteristic(kStatusUuid);
    if (command == nullptr || status == nullptr || !command->canWrite() ||
        !status->canNotify())
        return false;

    status->registerForNotify(handleNotification, true);
    portENTER_CRITICAL(&gLinksMux);
    link.command = command;
    link.connected = true;
    link.sync_samples = 0;
    link.node_state = NodeState::boot;
    portEXIT_CRITICAL(&gLinksMux);
    Serial.printf("connected node %u at %s\n", link.node_id,
                  link.address.c_str());
    return true;
}

void coordinatorScanComplete(BLEScanResults results)
{
    std::array<DiscoveredNode, kNodeCount> discovered{};
    std::array<bool, kNodeCount> found{};

    for (int i = 0; i < results.getCount(); ++i) {
        BLEAdvertisedDevice device = results.getDevice(i);
        if (!device.isAdvertisingService(BLEUUID(kServiceUuid)) ||
            !device.haveManufacturerData()) {
            continue;
        }
        const std::string manufacturer = device.getManufacturerData();
        if (manufacturer.size() < 2 ||
            static_cast<std::uint8_t>(manufacturer[0]) != 0xD2) {
            continue;
        }
        const std::uint8_t node_id =
            static_cast<std::uint8_t>(manufacturer[1]);
        if (node_id < 1 || node_id > kNodeCount)
            continue;

        DiscoveredNode& candidate = discovered[node_id - 1];
        candidate.node_id = node_id;
        candidate.address_type = device.getAddressType();
        const std::string address = device.getAddress().toString();
        std::snprintf(candidate.address, sizeof(candidate.address), "%s",
                      address.c_str());
        found[node_id - 1] = true;
    }

    std::uint32_t pending = 0;
    for (std::size_t i = 0; i < found.size(); ++i) {
        if (found[i])
            ++pending;
    }
    Serial.printf("SCAN complete: found %u wearable node(s)\n",
                  static_cast<unsigned>(pending));
    gPendingDiscoveredNodes.store(pending);
    for (std::size_t i = 0; i < found.size(); ++i) {
        if (!found[i])
            continue;
        if (gDiscoveredNodeQueue == nullptr ||
            xQueueSend(gDiscoveredNodeQueue, &discovered[i], 0) != pdTRUE) {
            gPendingDiscoveredNodes.fetch_sub(1);
        }
    }

    BLEDevice::getScan()->clearResults();
    gScanInProgress.store(false);
}

void coordinatorConnectionTask(void*)
{
    DiscoveredNode discovered{};
    for (;;) {
        if (xQueueReceive(gDiscoveredNodeQueue, &discovered,
                          portMAX_DELAY) != pdTRUE) {
            continue;
        }
        while (gScanInProgress.load())
            vTaskDelay(pdMS_TO_TICKS(10));

        LinkState* link = linkForNode(discovered.node_id);
        if (link != nullptr && !link->connected) {
            link->address = discovered.address;
            link->address_type = discovered.address_type;
            connectLink(*link);
        }
        gPendingDiscoveredNodes.fetch_sub(1);
    }
}

void discoverAndConnectNodes()
{
    if (gDiscoveredNodeQueue == nullptr) {
        Serial.println("SCAN rejected: discovery queue is unavailable");
        return;
    }
    if (gScanInProgress.load() || gPendingDiscoveredNodes.load() != 0) {
        Serial.println("SCAN ignored: scan or connection is already in progress");
        return;
    }

    bool need_scan = false;
    for (const auto& link : gLinks) {
        if (!link.connected) {
            need_scan = true;
            break;
        }
    }
    if (!need_scan) {
        Serial.println("SCAN skipped: all wearable nodes are already connected");
        return;
    }

    BLEScan* scan = BLEDevice::getScan();
    scan->setActiveScan(true);
    scan->setInterval(160);
    scan->setWindow(80);
    gScanInProgress.store(true);
    Serial.println("SCAN started: searching for wearable nodes for 4 seconds");
    if (!scan->start(4, coordinatorScanComplete, false)) {
        gScanInProgress.store(false);
        Serial.println("SCAN failed: BLE scanner did not start");
    }
}

void sendNextPing()
{
    const std::int64_t now_us = esp_timer_get_time();
    if (now_us - gLastPingUs < 200000)
        return;
    gLastPingUs = now_us;

    for (std::size_t attempts = 0; attempts < kNodeCount; ++attempts) {
        LinkState& link = gLinks[gNextPingLink];
        gNextPingLink = (gNextPingLink + 1) % kNodeCount;
        if (!link.connected)
            continue;
        WireMessage ping{};
        ping.type = static_cast<std::uint8_t>(MessageType::ping);
        ping.sequence = ++gSequence;
        ping.a = esp_timer_get_time();
        // PING is continuous best-effort telemetry. Do not block the BLE host
        // task waiting for an ATT write response; control messages below still
        // use acknowledged writes.
        sendMessage(link, ping, false);
        break;
    }
}

bool linkSynchronized(const LinkState& link, std::int64_t now_us)
{
    return link.connected && link.sync_samples >= kSyncSamplesRequired &&
           link.last_rtt_us <= kMaxAcceptedRttUs &&
           now_us - link.ref_coordinator_us < 2500000;
}

StatusLedEffect coordinatorStatusLedEffect()
{
    const std::int64_t now_us = esp_timer_get_time();
    const bool session_active = gLedSessionActive.load();
    const std::uint8_t participant_mask = gLedParticipantMask.load();
    bool any_connected = false;
    bool all_allowed = true;
    bool session_error = session_active && participant_mask == 0;
    bool all_running = session_active && participant_mask != 0;

    portENTER_CRITICAL(&gLinksMux);
    for (std::size_t i = 0; i < gLinks.size(); ++i) {
        const LinkState& link = gLinks[i];
        const bool participant =
            (participant_mask & (UINT8_C(1) << i)) != 0;
        if (session_active && participant) {
            const bool status_fresh = link.last_status_us > 0 &&
                now_us - link.last_status_us < 2000000;
            if (!link.connected || !status_fresh ||
                link.node_errors != error_none ||
                link.node_state == NodeState::fault) {
                session_error = true;
            }
            if (!link.connected || link.node_state != NodeState::running)
                all_running = false;
        }
        if (!link.connected)
            continue;
        any_connected = true;
        if (!linkSynchronized(link, now_us) ||
            link.node_errors != error_none ||
            link.node_state != NodeState::idle) {
            all_allowed = false;
        }
    }
    portEXIT_CRITICAL(&gLinksMux);

    if (session_active) {
        if (session_error)
            return StatusLedEffect::red_blink;
        return all_running ? StatusLedEffect::green_blink
                           : StatusLedEffect::blue_solid;
    }
    return any_connected && all_allowed
        ? StatusLedEffect::blue_solid
        : StatusLedEffect::boot_dim_red;
}

void coordinatorStatusLedTask(void*)
{
    for (;;) {
        serviceStatusLed(coordinatorStatusLedEffect());
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

std::int64_t coordinatorToLocal(const LinkState& link,
                                 std::int64_t coordinator_us)
{
    return static_cast<std::int64_t>(std::llround(
        link.ref_local_us +
        (coordinator_us - link.ref_coordinator_us) * link.local_rate));
}

bool validUtcNs(std::int64_t utc_ns)
{
    constexpr std::int64_t kMinUtcNs = 1577836800000000000LL;  // 2020-01-01
    constexpr std::int64_t kMaxUtcNs = 4102444800000000000LL;  // 2100-01-01
    return utc_ns >= kMinUtcNs && utc_ns < kMaxUtcNs;
}

bool allLinksTimeReady(std::int64_t now_us)
{
    bool any_connected = false;
    for (const auto& link : gLinks) {
        if (!link.connected)
            continue;
        any_connected = true;
        if (!linkSynchronized(link, now_us))
            return false;
    }
    return any_connected;
}

bool scheduleTimeDistribution(const UtcClockMap& map)
{
    const std::int64_t now_us = esp_timer_get_time();
    if (!map.valid || !validUtcNs(map.utc_ref_ns)) {
        Serial.println("TIME_ERROR invalid UTC clock map");
        return false;
    }
    if (!allLinksTimeReady(now_us)) {
        Serial.println("TIME_ERROR nodes are not synchronized; retry later");
        return false;
    }

    const std::int64_t target_us = now_us + kTimePlanLeadUs;
    const std::int64_t target_utc_ns =
        map.utc_ref_ns + (target_us - map.coordinator_ref_us) * 1000LL;
    if (!validUtcNs(target_utc_ns)) {
        Serial.println("TIME_ERROR computed target UTC is out of range");
        return false;
    }

    gUtcClockMap = map;
    gPendingTimeDistribution.active = true;
    gPendingTimeDistribution.sequence = map.sequence;
    gPendingTimeDistribution.coordinator_epoch_us = target_us;
    gPendingTimeDistribution.utc_epoch_ns = target_utc_ns;
    gPendingTimeDistribution.uncertainty_us = map.uncertainty_us;
    gPendingTimeDistribution.next_link = 0;
    gPendingTimeDistribution.sent_links = 0;
    Serial.printf("TIME_PLAN seq=%lu target_coordinator_us=%lld utc_ns=%lld\n",
                  static_cast<unsigned long>(map.sequence),
                  static_cast<long long>(target_us),
                  static_cast<long long>(target_utc_ns));
    return true;
}

void servicePendingTimeDistribution()
{
    if (!gPendingTimeDistribution.active)
        return;

    // START planning/arming owns the BLE control path for a few seconds.
    if (gStartPending && !gArmSent)
        return;

    const std::int64_t now_us = esp_timer_get_time();
    if (gPendingTimeDistribution.coordinator_epoch_us - now_us <
        kMinimumTimePlanMarginUs) {
        Serial.printf("TIME_ERROR seq=%lu missed distribution deadline\n",
                      static_cast<unsigned long>(
                          gPendingTimeDistribution.sequence));
        gPendingTimeDistribution.active = false;
        return;
    }

    if (gPendingTimeDistribution.next_link >= gLinks.size()) {
        Serial.printf("TIME_ACCEPT seq=%lu nodes=%u uncertainty_us=%lu\n",
                      static_cast<unsigned long>(
                          gPendingTimeDistribution.sequence),
                      static_cast<unsigned>(
                          gPendingTimeDistribution.sent_links),
                      static_cast<unsigned long>(
                          gPendingTimeDistribution.uncertainty_us));
        gPendingTimeDistribution.active = false;
        return;
    }

    while (gPendingTimeDistribution.next_link < gLinks.size() &&
           !gLinks[gPendingTimeDistribution.next_link].connected) {
        ++gPendingTimeDistribution.next_link;
    }
    if (gPendingTimeDistribution.next_link >= gLinks.size())
        return;

    LinkState& link = gLinks[gPendingTimeDistribution.next_link];
    if (!linkSynchronized(link, now_us)) {
        Serial.printf("TIME_ERROR seq=%lu node=%u lost synchronization\n",
                      static_cast<unsigned long>(
                          gPendingTimeDistribution.sequence),
                      link.node_id);
        gPendingTimeDistribution.active = false;
        return;
    }

    WireMessage time_sync{};
    time_sync.type = static_cast<std::uint8_t>(MessageType::time_sync);
    time_sync.sequence = gPendingTimeDistribution.sequence;
    time_sync.a = coordinatorToLocal(
        link, gPendingTimeDistribution.coordinator_epoch_us);
    time_sync.b = gPendingTimeDistribution.utc_epoch_ns;
    time_sync.x = gPendingTimeDistribution.uncertainty_us +
        static_cast<std::uint32_t>(std::max<std::int64_t>(
            0, link.last_rtt_us / 2));
    if (!sendMessage(link, time_sync, false)) {
        Serial.printf("TIME_ERROR seq=%lu node=%u BLE write failed\n",
                      static_cast<unsigned long>(
                          gPendingTimeDistribution.sequence),
                      link.node_id);
        gPendingTimeDistribution.active = false;
        return;
    }

    ++gPendingTimeDistribution.next_link;
    ++gPendingTimeDistribution.sent_links;
}

bool parseSimpleTimeCommand(const String& command, std::int64_t received_us,
                            UtcClockMap& map)
{
    unsigned long long seconds = 0;
    unsigned long microseconds = 0;
    char extra = '\0';
    if (std::sscanf(command.c_str(), "TIME %llu %lu %c",
                    &seconds, &microseconds, &extra) != 2 ||
        seconds < 1577836800ULL || seconds >= 4102444800ULL ||
        microseconds >= 1000000UL)
        return false;

    const std::int64_t utc_ns =
        static_cast<std::int64_t>(seconds) * 1000000000LL +
        static_cast<std::int64_t>(microseconds) * 1000LL;
    if (!validUtcNs(utc_ns))
        return false;

    map.valid = true;
    map.sequence = ++gFallbackTimeSequence;
    if (map.sequence == 0)
        map.sequence = ++gFallbackTimeSequence;
    map.coordinator_ref_us = received_us;
    map.utc_ref_ns = utc_ns;
    map.uncertainty_us = 20000;
    return true;
}

bool parsePreciseTimeSetCommand(const String& command,
                                std::int64_t received_us,
                                UtcClockMap& map)
{
    unsigned long sequence = 0;
    long long coordinator_ref_us = 0;
    long long utc_ref_ns = 0;
    unsigned long uncertainty_us = 0;
    char extra = '\0';
    if (std::sscanf(command.c_str(), "TIME_SET %lu %lld %lld %lu %c",
                    &sequence, &coordinator_ref_us, &utc_ref_ns,
                    &uncertainty_us, &extra) != 4 ||
        sequence == 0 || uncertainty_us > 1000000UL ||
        std::llabs(received_us - coordinator_ref_us) > 30000000LL ||
        !validUtcNs(utc_ref_ns))
        return false;

    map.valid = true;
    map.sequence = static_cast<std::uint32_t>(sequence);
    map.coordinator_ref_us = coordinator_ref_us;
    map.utc_ref_ns = utc_ref_ns;
    map.uncertainty_us = static_cast<std::uint32_t>(uncertainty_us);
    return true;
}

bool beginSession(std::uint32_t session_id)
{
    const std::int64_t now_us = esp_timer_get_time();
    std::size_t connected_nodes = 0;
    std::uint8_t participant_mask = 0;
    for (const auto& link : gLinks) {
        if (!link.connected)
            continue;
        ++connected_nodes;
        participant_mask |= UINT8_C(1) << (link.node_id - 1);
        if (!linkSynchronized(link, now_us) ||
            (link.node_state != NodeState::idle &&
             link.node_state != NodeState::fault)) {
            Serial.printf("START rejected: node %u connected=%d sync=%lu state=%s rtt=%lldus\n",
                          link.node_id, link.connected,
                          static_cast<unsigned long>(link.sync_samples),
                          stateName(link.node_state),
                          static_cast<long long>(link.last_rtt_us));
            return false;
        }
    }
    if (connected_nodes == 0) {
        Serial.println("START rejected: no wearable connected");
        return false;
    }

    const std::int64_t epoch_us = now_us + kPlanLeadUs;
    for (auto& link : gLinks) {
        if (!link.connected)
            continue;
        WireMessage plan{};
        plan.type = static_cast<std::uint8_t>(MessageType::plan);
        plan.sequence = ++gSequence;
        plan.session_id = session_id;
        plan.a = coordinatorToLocal(link, epoch_us);
        plan.b = static_cast<std::int64_t>(std::llround(
            kFramePeriodUs * link.local_rate * 4294967296.0));
        plan.x = kPulseWidthUs;
        if (!sendMessage(link, plan, true))
            return false;
    }

    gStartPending = true;
    gArmSent = false;
    gPendingSession = session_id;
    gPendingCoordinatorEpochUs = epoch_us;
    gLedParticipantMask.store(participant_mask);
    gLedSessionActive.store(true);
    Serial.printf("session %lu planned; common epoch=%lldus, waiting for %u Neo ACK(s)\n",
                  static_cast<unsigned long>(session_id),
                  static_cast<long long>(epoch_us),
                  static_cast<unsigned>(connected_nodes));
    return true;
}

void servicePendingStart()
{
    if (!gStartPending || gArmSent)
        return;

    bool all_ready = true;
    bool any_fault = false;
    bool any_connected = false;
    for (const auto& link : gLinks) {
        if (!link.connected)
            continue;
        any_connected = true;
        if (link.node_state == NodeState::fault)
            any_fault = true;
        if (link.node_session != gPendingSession ||
            link.node_state != NodeState::ready) {
            all_ready = false;
        }
    }
    if (!any_connected)
        all_ready = false;

    const std::int64_t now_us = esp_timer_get_time();
    if (!all_ready && (any_fault ||
        gPendingCoordinatorEpochUs - now_us < kMinimumArmMarginUs)) {
        Serial.println(any_fault
            ? "START aborted: a node reported FAULT"
            : "START aborted: not all nodes became READY before deadline");
        for (auto& link : gLinks) {
            WireMessage abort{};
            abort.type = static_cast<std::uint8_t>(MessageType::abort_session);
            abort.sequence = ++gSequence;
            abort.session_id = gPendingSession;
            sendMessage(link, abort, true);
        }
        gStartPending = false;
        gLedSessionActive.store(false);
        gLedParticipantMask.store(0);
        return;
    }

    if (!all_ready)
        return;

    for (auto& link : gLinks) {
        if (!link.connected)
            continue;
        WireMessage arm{};
        arm.type = static_cast<std::uint8_t>(MessageType::arm);
        arm.sequence = ++gSequence;
        arm.session_id = gPendingSession;
        sendMessage(link, arm, true);
    }
    gArmSent = true;
    Serial.printf("session %lu ARMED on connected nodes\n",
                  static_cast<unsigned long>(gPendingSession));
}

void stopSession()
{
    if (!gStartPending && gPendingSession == 0) {
        Serial.println("no active session");
        return;
    }
    const std::int64_t coordinator_stop_us = esp_timer_get_time() + 700000;
    for (auto& link : gLinks) {
        if (!link.connected)
            continue;
        WireMessage stop{};
        stop.type = static_cast<std::uint8_t>(MessageType::stop);
        stop.sequence = ++gSequence;
        stop.session_id = gPendingSession;
        stop.a = coordinatorToLocal(link, coordinator_stop_us);
        sendMessage(link, stop, true);
    }
    Serial.printf("STOP scheduled at coordinator=%lldus\n",
                  static_cast<long long>(coordinator_stop_us));
    gStartPending = false;
    gArmSent = false;
    gLedSessionActive.store(false);
    gLedParticipantMask.store(0);
}

void abortSession()
{
    for (auto& link : gLinks) {
        if (!link.connected)
            continue;
        WireMessage abort{};
        abort.type = static_cast<std::uint8_t>(MessageType::abort_session);
        abort.sequence = ++gSequence;
        abort.session_id = gPendingSession;
        sendMessage(link, abort, true);
    }
    gStartPending = false;
    gArmSent = false;
    gPendingSession = 0;
    gLedSessionActive.store(false);
    gLedParticipantMask.store(0);
    Serial.println("ABORT sent to all nodes");
}

void printCoordinatorStatus()
{
    const std::int64_t now_us = esp_timer_get_time();
    for (const auto& link : gLinks) {
        const double ppm = (link.local_rate - 1.0) * 1000000.0;
        Serial.printf("node=%u connected=%d state=%s session=%lu error=0x%08lx "
                      "samples=%lu rtt=%lldus rate=%+.2fppm fresh=%lldms\n",
                      link.node_id, link.connected, stateName(link.node_state),
                      static_cast<unsigned long>(link.node_session),
                      static_cast<unsigned long>(link.node_errors),
                      static_cast<unsigned long>(link.sync_samples),
                      static_cast<long long>(link.last_rtt_us), ppm,
                      static_cast<long long>((now_us - link.ref_coordinator_us) / 1000));
    }
    if (gUtcClockMap.valid) {
        Serial.printf("utc_map=LOCKED seq=%lu age=%lldms uncertainty=%luus pending=%d\n",
                      static_cast<unsigned long>(gUtcClockMap.sequence),
                      static_cast<long long>(
                          (now_us - gUtcClockMap.coordinator_ref_us) / 1000),
                      static_cast<unsigned long>(
                          gUtcClockMap.uncertainty_us),
                      gPendingTimeDistribution.active);
    } else {
        Serial.println("utc_map=UNLOCKED");
    }
}

void processCoordinatorCommand(const String& command)
{
    const std::int64_t received_us = esp_timer_get_time();
    if (command.startsWith("TIME_QUERY")) {
        unsigned long sequence = 0;
        char extra = '\0';
        if (std::sscanf(command.c_str(), "TIME_QUERY %lu %c",
                        &sequence, &extra) != 1 || sequence == 0) {
            Serial.println("TIME_ERROR usage: TIME_QUERY <sequence>");
            return;
        }
        const std::int64_t transmit_us = esp_timer_get_time();
        Serial.printf("TIME_REPLY %lu %lld %lld\n", sequence,
                      static_cast<long long>(received_us),
                      static_cast<long long>(transmit_us));
    } else if (command.startsWith("TIME_SET")) {
        UtcClockMap map{};
        if (!parsePreciseTimeSetCommand(command, received_us, map)) {
            Serial.println(
                "TIME_ERROR usage: TIME_SET <seq> <coordinator_ref_us> "
                "<utc_ref_ns> <uncertainty_us>");
            return;
        }
        scheduleTimeDistribution(map);
    } else if (command.startsWith("TIME ")) {
        UtcClockMap map{};
        if (!parseSimpleTimeCommand(command, received_us, map)) {
            Serial.println("TIME_ERROR usage: TIME <unix_sec> <usec>");
            return;
        }
        scheduleTimeDistribution(map);
    } else if (command.equalsIgnoreCase("SCAN")) {
        discoverAndConnectNodes();
    } else if (command.startsWith("START")) {
        const std::uint32_t fallback =
            static_cast<std::uint32_t>(esp_timer_get_time() / 1000);
        beginSession(parseSession(command, fallback == 0 ? 1 : fallback));
    } else if (command.equalsIgnoreCase("STOP")) {
        stopSession();
    } else if (command.equalsIgnoreCase("ABORT")) {
        abortSession();
    } else if (command.equalsIgnoreCase("STATUS")) {
        printCoordinatorStatus();
    } else if (command.length() > 0) {
        Serial.println(
            "commands: SCAN, START <session>, STOP, ABORT, STATUS, "
            "TIME <unix_sec> <usec>, TIME_QUERY <seq>, "
            "TIME_SET <seq> <coordinator_ref_us> <utc_ref_ns> <uncertainty_us>");
    }
}

}  // namespace

void setup()
{
    Serial.begin(115200);
    delay(300);
    initializeStatusLed();
    xTaskCreatePinnedToCore(coordinatorStatusLedTask, "status_led", 2048,
                            nullptr, 2, nullptr, 0);
    BLEDevice::init("Mode2Coordinator");
    BLEDevice::setCustomGattcHandler(coordinatorGattcEventHandler);
    BLEDevice::setMTU(185);
    BLEDevice::setPower(ESP_PWR_LVL_P9);
    gDiscoveredNodeQueue = xQueueCreate(kNodeCount, sizeof(DiscoveredNode));
    if (gDiscoveredNodeQueue != nullptr) {
        xTaskCreatePinnedToCore(coordinatorConnectionTask, "ble_connect", 4096,
                                nullptr, 2, nullptr, 1);
    } else {
        Serial.println("FATAL: BLE discovery queue initialization failed");
    }
    Serial.printf("Mode2+Time coordinator ready; SCAN discovers nodes 1..%u on demand\n",
                  static_cast<unsigned>(kNodeCount));
    Serial.println(
        "commands: SCAN, START <session>, STOP, ABORT, STATUS, "
        "TIME <unix_sec> <usec>, TIME_QUERY <seq>, TIME_SET ...");
}

void loop()
{
    sendNextPing();
    servicePendingStart();
    processCoordinatorCommand(readUsbLine());
    servicePendingTimeDistribution();

    static std::uint32_t last_status_ms = 0;
    if (millis() - last_status_ms >= 5000) {
        last_status_ms = millis();
        printCoordinatorStatus();
    }
    delay(2);
}

#endif  // MODE2_ROLE_COORDINATOR
