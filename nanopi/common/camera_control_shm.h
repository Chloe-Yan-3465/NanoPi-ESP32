#pragma once

#include <cstddef>
#include <cstdint>

// Compatible with nanoPi_v2/gpio/uart_camera_receiver.cpp.  The camera
// process creates this object; the UART receiver opens it afterwards.
static constexpr const char* CAMERA_CONTROL_SHM_NAME = "/yuv_ep_flag";
static constexpr std::uint32_t CAMERA_CONTROL_SHM_MAGIC = 0x43414D35U;
static constexpr std::uint32_t CAMERA_CONTROL_SHM_VERSION = 1U;

enum CameraControlCommand : std::uint32_t
{
    CAMERA_COMMAND_NONE = 0,
    CAMERA_COMMAND_START = 1,
    CAMERA_COMMAND_STOP = 2,
    CAMERA_COMMAND_STATUS = 3
};

enum CameraControlState : std::uint32_t
{
    CAMERA_STATE_IDLE = 0,
    CAMERA_STATE_STARTING = 1,
    CAMERA_STATE_RUNNING = 2,
    CAMERA_STATE_STOPPING = 3,
    CAMERA_STATE_ERROR = 4
};

enum CameraControlResult : std::int32_t
{
    CAMERA_RESULT_OK = 0,
    CAMERA_RESULT_INVALID_COMMAND = 1,
    CAMERA_RESULT_BUSY = 2,
    CAMERA_RESULT_SESSION_MISMATCH = 3,
    CAMERA_RESULT_EP_CREATE_FAILED = 4,
    CAMERA_RESULT_CAMERA_ERROR = 5,
    CAMERA_RESULT_INTERNAL_ERROR = 6
};

struct CameraControlSharedMemory
{
    std::uint32_t magic;
    std::uint32_t version;

    volatile std::uint32_t request_seq;
    volatile std::uint32_t response_seq;
    volatile std::uint32_t command;
    volatile std::uint32_t session_id;

    volatile std::uint32_t state;
    volatile std::uint64_t frames_written;
    volatile std::int32_t result_code;

    char error[128];
    char ep_path[512];
};

static inline std::uint32_t camera_shm_load_u32(
    const volatile std::uint32_t* value)
{
    return __atomic_load_n(value, __ATOMIC_ACQUIRE);
}

static inline void camera_shm_store_u32(
    volatile std::uint32_t* value, std::uint32_t new_value)
{
    __atomic_store_n(value, new_value, __ATOMIC_RELEASE);
}

static inline std::uint64_t camera_shm_load_u64(
    const volatile std::uint64_t* value)
{
    return __atomic_load_n(value, __ATOMIC_ACQUIRE);
}

static inline void camera_shm_store_u64(
    volatile std::uint64_t* value, std::uint64_t new_value)
{
    __atomic_store_n(value, new_value, __ATOMIC_RELEASE);
}

static inline std::int32_t camera_shm_load_i32(
    const volatile std::int32_t* value)
{
    return __atomic_load_n(value, __ATOMIC_ACQUIRE);
}

static inline void camera_shm_store_i32(
    volatile std::int32_t* value, std::int32_t new_value)
{
    __atomic_store_n(value, new_value, __ATOMIC_RELEASE);
}

static_assert(offsetof(CameraControlSharedMemory, request_seq) == 8,
              "Unexpected shared-memory layout");
