#pragma once

#include "usb_handler.hpp"

class UsbHandlerXvf3000 : public UsbHandler
{
public:
    static const uint16_t DEFAULT_PID = 0x0018;

    UsbHandlerXvf3000(uint16_t vid, uint16_t pid, int timeout_ms);
    ~UsbHandlerXvf3000() override;

    int32_t readDoaAngle() override;

    bool ledOff() override;
    bool ledSetMode(int mode) override;
    bool ledSetBrightness(uint8_t brightness) override;
    bool ledSetColorAll(uint8_t r, uint8_t g, uint8_t b) override;
    bool ledSetColorManual(const std::vector<LedColor> &colors) override;
    bool ledSetColorPalette(const LedColor &a, const LedColor &b) override;

private:
    static const uint16_t LED_WINDEX   = 0x1C;
    static const uint16_t DOA_PARAM_ID = 21;
    static const uint8_t  DOA_CMD      = 0xC0;

    int writeCmd(uint8_t cmd, const std::vector<uint8_t> &data);
};