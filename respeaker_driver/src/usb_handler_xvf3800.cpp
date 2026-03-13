#include "usb_handler_xvf3800.hpp"

#include <chrono>
#include <thread>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

UsbHandlerXvf3800::UsbHandlerXvf3800(uint16_t vid, uint16_t pid, int timeout_ms)
    : UsbHandler(vid, pid, timeout_ms), speech_detected_(false)
{
}

UsbHandlerXvf3800::~UsbHandlerXvf3800()
{
}

// ================================================================
//  Low-level read / write
// ================================================================

bool UsbHandlerXvf3800::writeParam(uint16_t resid, uint16_t cmdid,
                                    const std::vector<uint8_t> &payload)
{
    int err = libusb_control_transfer(
        dev_,
        LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
        0x00,
        cmdid,
        resid,
        const_cast<uint8_t *>(payload.data()),
        static_cast<uint16_t>(payload.size()),
        timeout_);

    return (err == static_cast<int>(payload.size()));
}

std::vector<uint8_t> UsbHandlerXvf3800::readParam(uint16_t resid, uint16_t cmdid,
                                                    int total_response_len)
{
    uint16_t wvalue = static_cast<uint16_t>(0x80u | cmdid);
    std::vector<unsigned char> buf(total_response_len);

    for (int attempt = 0; attempt < MAX_RETRIES; ++attempt)
    {
        int ret = libusb_control_transfer(
            dev_,
            LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
            0x00,
            wvalue,
            resid,
            buf.data(),
            static_cast<uint16_t>(buf.size()),
            timeout_);

        if (ret < 1)
            return std::vector<uint8_t>();

        if (buf[0] == STATUS_OK)
        {
            std::vector<uint8_t> result(buf.begin() + 1, buf.begin() + ret);
            return result;
        }

        if (buf[0] == STATUS_RETRY)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        return std::vector<uint8_t>();
    }

    return std::vector<uint8_t>();
}

// ================================================================
//  DOA
//
//  Используем AEC_AZIMUTH_VALUES (resid=33, cmdid=75):
//    4 float (radians): beam1, beam2, free-running, auto-select
//    Берём index=3 (auto-select beam)
//    Значение: 0 .. 2*PI, отсчёт по часовой стрелке
//
//  Для speech detection читаем DOA_VALUE (resid=20, cmdid=18):
//    2 uint16: doa(0..359), speech(0/1)
//    Используем только поле speech
// ================================================================

int32_t UsbHandlerXvf3800::readDoaAngle()
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (dev_ == nullptr)
        return 0;

    int32_t doa_degrees = 0;

    // ── 1. Читаем AEC_AZIMUTH_VALUES для DOA ──
    // Ответ (без статуса): 4 float = 16 байт
    // [0..3]   beam 1
    // [4..7]   beam 2
    // [8..11]  free-running beam
    // [12..15] auto-select beam  <-- нужен этот
    std::vector<uint8_t> azimuth_data = readParam(
        RESID_AEC, CMD_AEC_AZIMUTH_VALUES, AEC_AZIMUTH_RESPONSE_LEN);

    int required_bytes = (AEC_AZIMUTH_AUTO_SELECT_INDEX + 1) * 4;  // 16

    if (static_cast<int>(azimuth_data.size()) >= required_bytes)
    {
        int offset = AEC_AZIMUTH_AUTO_SELECT_INDEX * 4;  // 12
        float azimuth_rad = unpackLeFloat(&azimuth_data[offset]);

        // Проверка на NaN / Inf (может вернуться если нет речи)
        if (!std::isnan(azimuth_rad) && !std::isinf(azimuth_rad))
        {
            // Перевод радиан → градусы
            double degrees = azimuth_rad * 180.0 / M_PI;

            // Округление
            doa_degrees = static_cast<int32_t>(std::round(degrees));

            // Нормализация в диапазон 0..359
            doa_degrees = doa_degrees % 360;
            if (doa_degrees < 0)
            {
                doa_degrees += 360;
            }
        }
    }

    // // ── 2. Читаем DOA_VALUE для speech detection ──
    // // Ответ (без статуса): 2 uint16 = 4 байта
    // // [0..1] doa (не используем)
    // // [2..3] speech (0 или 1)
    // std::vector<uint8_t> doa_data = readParam(
    //     RESID_GPO, CMD_DOA_VALUE, DOA_VALUE_RESPONSE_LEN);

    // if (doa_data.size() >= 4)
    // {
    //     uint16_t speech = unpackLeU16(&doa_data[2]);
    //     speech_detected_.store(speech != 0);
    // }

    return -doa_degrees;
}


// ================================================================
//  LED (без изменений)
// ================================================================

bool UsbHandlerXvf3800::ledOff()
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (dev_ == nullptr)
        return false;

    std::vector<uint8_t> data = {0};
    return writeParam(RESID_GPO, CMD_LED_EFFECT, data);
}

bool UsbHandlerXvf3800::ledSetMode(int mode)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (dev_ == nullptr)
        return false;

    if (mode < 0 || mode > 5)
        return false;

    if (mode==1) {mode = 4;}
    else if (mode==4) {mode = 1;}

    std::vector<uint8_t> data = {static_cast<uint8_t>(mode)};
    return writeParam(RESID_GPO, CMD_LED_EFFECT, data);
}

bool UsbHandlerXvf3800::ledSetBrightness(uint8_t brightness)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (dev_ == nullptr)
        return false;

    std::vector<uint8_t> data = {brightness};
    return writeParam(RESID_GPO, CMD_LED_BRIGHTNESS, data);
}

bool UsbHandlerXvf3800::ledSetColorAll(uint8_t r, uint8_t g, uint8_t b)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (dev_ == nullptr)
        return false;

    uint8_t payload[4];
    packLeU32(payload, rgbToU32(r, g, b));

    std::vector<uint8_t> data = {payload[0], payload[1], payload[2], payload[3]};
    return writeParam(RESID_GPO, CMD_LED_COLOR, data);
}

bool UsbHandlerXvf3800::ledSetColorManual(const std::vector<LedColor> &colors)
{
    if (static_cast<int>(colors.size()) != NUM_LEDS)
        return false;

    std::lock_guard<std::mutex> lock(mutex_);

    if (dev_ == nullptr)
        return false;

    std::vector<uint8_t> payload(48);
    for (int i = 0; i < NUM_LEDS; ++i)
    {
        packLeU32(&payload[i * 4], rgbToU32(colors[i].r, colors[i].g, colors[i].b));
    }

    return writeParam(RESID_GPO, CMD_LED_RING_COLOR, payload);
}

bool UsbHandlerXvf3800::ledSetColorPalette(const LedColor &a, const LedColor &b)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (dev_ == nullptr)
        return false;

    std::vector<uint8_t> payload(8);
    packLeU32(&payload[0], rgbToU32(a.r, a.g, a.b));
    packLeU32(&payload[4], rgbToU32(b.r, b.g, b.b));

    return writeParam(RESID_GPO, CMD_LED_DOA_COLOR, payload);
}