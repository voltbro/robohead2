#include "usb_handler_xvf3000.hpp"

UsbHandlerXvf3000::UsbHandlerXvf3000(uint16_t vid, uint16_t pid, int timeout_ms)
    : UsbHandler(vid, pid, timeout_ms)
{
}

UsbHandlerXvf3000::~UsbHandlerXvf3000()
{
}

// ================================================================
//  Low-level LED command
// ================================================================

int UsbHandlerXvf3000::writeCmd(uint8_t cmd, const std::vector<uint8_t> &data)
{
    int err = libusb_control_transfer(
        dev_,
        LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE | LIBUSB_ENDPOINT_OUT,
        0x00,
        cmd,
        LED_WINDEX,
        const_cast<uint8_t *>(data.data()),
        static_cast<uint16_t>(data.size()),
        timeout_);

    if (err == static_cast<int>(data.size()))
        return 0;
    else
        return -1;
}

// ================================================================
//  DOA
// ================================================================

int32_t UsbHandlerXvf3000::readDoaAngle()
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (dev_ == nullptr)
        return 0;

    unsigned char buf[8];

    int ret = libusb_control_transfer(
        dev_,
        LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
        0x00,
        DOA_CMD,
        DOA_PARAM_ID,
        buf, 8, timeout_);

    if (ret != 8)
        return 0;

    int32_t raw_angle = unpackLeI32(buf);

    // Поворот на 90° по часовой стрелке
    int32_t rotated = (raw_angle + 90) % 360;

    return rotated;
}

// ================================================================
//  LED
// ================================================================

bool UsbHandlerXvf3000::ledOff()
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (dev_ == nullptr)
        return false;

    std::vector<uint8_t> data = {0, 0, 0, 0};
    return writeCmd(0x01, data) == 0;
}

bool UsbHandlerXvf3000::ledSetMode(int mode)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (dev_ == nullptr)
        return false;

    switch (mode)
    {
    case 0:
    {
        std::vector<uint8_t> data = {0, 0, 0, 0};
        return writeCmd(0x01, data) == 0;
    }
    case 1:
    {
        std::vector<uint8_t> data = {0};
        return writeCmd(0, data) == 0;
    }
    case 2:
    {
        std::vector<uint8_t> data = {0};
        return writeCmd(2, data) == 0;
    }
    case 3:
    {
        std::vector<uint8_t> data = {0};
        return writeCmd(3, data) == 0;
    }
    case 4:
    {
        std::vector<uint8_t> data = {0};
        return writeCmd(4, data) == 0;
    }
    case 5:
    {
        std::vector<uint8_t> data = {0};
        return writeCmd(5, data) == 0;
    }
    default:
        return false;
    }
}

bool UsbHandlerXvf3000::ledSetBrightness(uint8_t brightness)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (dev_ == nullptr)
        return false;

    if (brightness > 31)
        return false;

    std::vector<uint8_t> data = {brightness};
    return writeCmd(0x20, data) == 0;
}

bool UsbHandlerXvf3000::ledSetColorAll(uint8_t r, uint8_t g, uint8_t b)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (dev_ == nullptr)
        return false;

    std::vector<uint8_t> data = {r, g, b, 0};
    return writeCmd(0x01, data) == 0;
}

bool UsbHandlerXvf3000::ledSetColorManual(const std::vector<LedColor> &colors)
{
    if (static_cast<int>(colors.size()) != NUM_LEDS)
        return false;

    std::lock_guard<std::mutex> lock(mutex_);

    if (dev_ == nullptr)
        return false;

    std::vector<uint8_t> data(48, 0);
    for (int i = 0; i < NUM_LEDS; ++i)
    {
        data[i * 4]     = colors[i].r;
        data[i * 4 + 1] = colors[i].g;
        data[i * 4 + 2] = colors[i].b;
        data[i * 4 + 3] = 0;
    }

    return writeCmd(0x06, data) == 0;
}

bool UsbHandlerXvf3000::ledSetColorPalette(const LedColor &a, const LedColor &b)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (dev_ == nullptr)
        return false;

    std::vector<uint8_t> data = {a.r, a.g, a.b, 0,
                                  b.r, b.g, b.b, 0};
    return writeCmd(0x21, data) == 0;
}