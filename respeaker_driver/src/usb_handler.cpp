#include "usb_handler.hpp"
#include "usb_handler_xvf3000.hpp"
#include "usb_handler_xvf3800.hpp"

#include <algorithm>
#include <cctype>
#include <cstring>
// ================================================================
//  Byte helpers
// ================================================================

float UsbHandler::unpackLeFloat(const uint8_t *src)
{
    float result;
    std::memcpy(&result, src, sizeof(float));
    return result;
}

void UsbHandler::packLeU32(uint8_t *dst, uint32_t val)
{
    dst[0] = static_cast<uint8_t>(val);
    dst[1] = static_cast<uint8_t>(val >> 8);
    dst[2] = static_cast<uint8_t>(val >> 16);
    dst[3] = static_cast<uint8_t>(val >> 24);
}

uint16_t UsbHandler::unpackLeU16(const uint8_t *src)
{
    return static_cast<uint16_t>(src[0]) |
           (static_cast<uint16_t>(src[1]) << 8);
}

int32_t UsbHandler::unpackLeI32(const uint8_t *src)
{
    return static_cast<int32_t>(
        static_cast<uint32_t>(src[0]) |
        (static_cast<uint32_t>(src[1]) << 8) |
        (static_cast<uint32_t>(src[2]) << 16) |
        (static_cast<uint32_t>(src[3]) << 24));
}

uint32_t UsbHandler::rgbToU32(uint8_t r, uint8_t g, uint8_t b)
{
    return (static_cast<uint32_t>(r) << 16) |
           (static_cast<uint32_t>(g) << 8) |
            static_cast<uint32_t>(b);
}

// ================================================================
//  Constructor / Destructor
// ================================================================

UsbHandler::UsbHandler(uint16_t vid, uint16_t pid, int timeout_ms)
    : vid_(vid), pid_(pid), timeout_(timeout_ms), ctx_(nullptr), dev_(nullptr)
{
}

UsbHandler::~UsbHandler()
{
    close();
}

// ================================================================
//  Getters
// ================================================================

uint16_t UsbHandler::getVid() const { return vid_; }
uint16_t UsbHandler::getPid() const { return pid_; }
bool     UsbHandler::isOpen() const { return dev_ != nullptr; }

bool UsbHandler::isSpeechDetected() const { return false; }

// ================================================================
//  init / close / reset
// ================================================================

bool UsbHandler::init()
{
    std::lock_guard<std::mutex> lock(mutex_);

    int err = libusb_init(&ctx_);
    if (err < 0)
        return false;

    dev_ = libusb_open_device_with_vid_pid(ctx_, vid_, pid_);
    if (dev_ == nullptr)
    {
        libusb_exit(ctx_);
        ctx_ = nullptr;
        return false;
    }

    if (libusb_kernel_driver_active(dev_, 0) == 1)
    {
        libusb_detach_kernel_driver(dev_, 0);
    }

    err = libusb_claim_interface(dev_, 0);
    if (err < 0)
    {
        libusb_close(dev_);
        dev_ = nullptr;
        libusb_exit(ctx_);
        ctx_ = nullptr;
        return false;
    }

    return true;
}

void UsbHandler::close()
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (dev_ != nullptr)
    {
        if (libusb_kernel_driver_active(dev_, 0) == 0)
        {
            libusb_attach_kernel_driver(dev_, 0);
        }

        libusb_release_interface(dev_, 0);
        libusb_close(dev_);
        dev_ = nullptr;
    }

    if (ctx_ != nullptr)
    {
        libusb_exit(ctx_);
        ctx_ = nullptr;
    }
}

bool UsbHandler::resetDevice()
{
    libusb_context *tmp_ctx = nullptr;
    if (libusb_init(&tmp_ctx) < 0)
        return false;

    libusb_device_handle *tmp_dev =
        libusb_open_device_with_vid_pid(tmp_ctx, vid_, pid_);

    if (tmp_dev == nullptr)
    {
        libusb_exit(tmp_ctx);
        return false;
    }

    if (libusb_kernel_driver_active(tmp_dev, 0) == 1)
    {
        libusb_detach_kernel_driver(tmp_dev, 0);
    }

    int err = libusb_reset_device(tmp_dev);

    if (libusb_kernel_driver_active(tmp_dev, 0) == 0)
    {
        libusb_attach_kernel_driver(tmp_dev, 0);
    }

    libusb_close(tmp_dev);
    libusb_exit(tmp_ctx);

    return (err == LIBUSB_SUCCESS);
}

// ================================================================
//  Auto-detection
// ================================================================

bool UsbHandler::probeDevice(uint16_t vid, uint16_t pid)
{
    libusb_context *ctx = nullptr;
    if (libusb_init(&ctx) < 0)
        return false;

    libusb_device_handle *dev =
        libusb_open_device_with_vid_pid(ctx, vid, pid);

    bool found = (dev != nullptr);

    if (dev != nullptr)
        libusb_close(dev);

    libusb_exit(ctx);
    return found;
}

UsbHandler::DetectedDevice UsbHandler::autoDetect(uint16_t vid_override,
                                                   uint16_t pid_override)
{
    struct KnownDevice
    {
        const char *version;
        uint16_t    vid;
        uint16_t    pid;
    };

    const KnownDevice known[] = {
        {"xvf3800", 0x2886, UsbHandlerXvf3800::DEFAULT_PID},
        {"xvf3000", 0x2886, UsbHandlerXvf3000::DEFAULT_PID},
    };
    const int known_count = 2;

    uint16_t vid = (vid_override != 0) ? vid_override : 0x2886;

    // Конкретный PID задан — пробуем только его
    if (pid_override != 0)
    {
        if (probeDevice(vid, pid_override))
        {
            for (int i = 0; i < known_count; ++i)
            {
                if (known[i].pid == pid_override)
                    return DetectedDevice(known[i].version, vid, pid_override, true);
            }
            // Неизвестный PID → считаем xvf3000
            return DetectedDevice("xvf3000", vid, pid_override, true);
        }
        return DetectedDevice("", vid, pid_override, false);
    }

    // Перебор известных устройств
    for (int i = 0; i < known_count; ++i)
    {
        if (probeDevice(vid, known[i].pid))
            return DetectedDevice(known[i].version, vid, known[i].pid, true);
    }

    return DetectedDevice("", vid, 0, false);
}

// ================================================================
//  Factory
// ================================================================

std::unique_ptr<UsbHandler> UsbHandler::create(const std::string &version,
                                                uint16_t vid,
                                                uint16_t pid,
                                                int timeout_ms)
{
    // Приводим к нижнему регистру
    std::string v = version;
    for (size_t i = 0; i < v.size(); ++i)
        v[i] = static_cast<char>(std::tolower(v[i]));

    if (v == "xvf3000")
    {
        uint16_t p = (pid != 0) ? pid : UsbHandlerXvf3000::DEFAULT_PID;
        return std::make_unique<UsbHandlerXvf3000>(vid, p, timeout_ms);
    }

    if (v == "xvf3800")
    {
        uint16_t p = (pid != 0) ? pid : UsbHandlerXvf3800::DEFAULT_PID;
        return std::make_unique<UsbHandlerXvf3800>(vid, p, timeout_ms);
    }

    return nullptr;
}