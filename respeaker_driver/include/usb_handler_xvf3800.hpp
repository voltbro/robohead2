#pragma once

#include "usb_handler.hpp"
#include <atomic>

class UsbHandlerXvf3800 : public UsbHandler
{
public:
    static const uint16_t DEFAULT_PID = 0x001A;

    UsbHandlerXvf3800(uint16_t vid, uint16_t pid, int timeout_ms);
    ~UsbHandlerXvf3800() override;

    int32_t readDoaAngle() override;

    bool ledOff() override;
    bool ledSetMode(int mode) override;
    bool ledSetBrightness(uint8_t brightness) override;
    bool ledSetColorAll(uint8_t r, uint8_t g, uint8_t b) override;
    bool ledSetColorManual(const std::vector<LedColor> &colors) override;
    bool ledSetColorPalette(const LedColor &a, const LedColor &b) override;

private:
    // AEC servicer (DOA через beamformer)
    static const uint16_t RESID_AEC                     = 33;
    static const uint16_t CMD_AEC_AZIMUTH_VALUES        = 75;
    static const int      AEC_AZIMUTH_COUNT             = 4;   // 4 float значения
    static const int      AEC_AZIMUTH_AUTO_SELECT_INDEX = 3;   // индекс auto-select beam
    // Размер ответа: 1(status) + 4*sizeof(float) = 17 байт
    static const int      AEC_AZIMUTH_RESPONSE_LEN      = 17;

    // GPO servicer (LED + speech detection)
    static const uint16_t RESID_GPO          = 20;
    static const uint16_t CMD_LED_EFFECT     = 12;
    static const uint16_t CMD_LED_BRIGHTNESS = 13;
    static const uint16_t CMD_LED_COLOR      = 16;
    static const uint16_t CMD_LED_DOA_COLOR  = 17;
    static const uint16_t CMD_DOA_VALUE      = 18;
    static const uint16_t CMD_LED_RING_COLOR = 19;
    // Размер ответа DOA_VALUE: 1(status) + 2*sizeof(uint16) = 5 байт
    static const int      DOA_VALUE_RESPONSE_LEN = 5;

    // Retry protocol
    static const uint8_t STATUS_OK    = 0;
    static const uint8_t STATUS_RETRY = 64;
    static const int     MAX_RETRIES  = 100;

    bool writeParam(uint16_t resid, uint16_t cmdid,
                    const std::vector<uint8_t> &payload);

    std::vector<uint8_t> readParam(uint16_t resid, uint16_t cmdid,
                                   int total_response_len);

    std::atomic<bool> speech_detected_;
};