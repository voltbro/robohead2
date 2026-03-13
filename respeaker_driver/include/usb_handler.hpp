#pragma once

#include <libusb-1.0/libusb.h>
#include <cstdint>
#include <vector>
#include <mutex>
#include <memory>
#include <string>

class UsbHandler
{
public:
    static const int NUM_LEDS = 12;

    struct LedColor
    {
        uint8_t r;
        uint8_t g;
        uint8_t b;

        LedColor() : r(0), g(0), b(0) {}
        LedColor(uint8_t r_, uint8_t g_, uint8_t b_) : r(r_), g(g_), b(b_) {}
    };

    struct DetectedDevice
    {
        std::string version;
        uint16_t    vid;
        uint16_t    pid;
        bool        found;

        DetectedDevice() : vid(0), pid(0), found(false) {}
        DetectedDevice(const std::string &v, uint16_t vi, uint16_t pi, bool f)
            : version(v), vid(vi), pid(pi), found(f) {}
    };

    UsbHandler(uint16_t vid, uint16_t pid, int timeout_ms);
    virtual ~UsbHandler();

    UsbHandler(const UsbHandler &) = delete;
    UsbHandler &operator=(const UsbHandler &) = delete;

    bool init();
    void close();
    bool resetDevice();

    // DOA
    virtual int32_t readDoaAngle() = 0;
    virtual bool isSpeechDetected() const;

    // LED
    virtual bool ledOff() = 0;
    virtual bool ledSetMode(int mode) = 0;
    virtual bool ledSetBrightness(uint8_t brightness) = 0;
    virtual bool ledSetColorAll(uint8_t r, uint8_t g, uint8_t b) = 0;
    virtual bool ledSetColorManual(const std::vector<LedColor> &colors) = 0;
    virtual bool ledSetColorPalette(const LedColor &a, const LedColor &b) = 0;

    // Getters
    uint16_t getVid() const;
    uint16_t getPid() const;
    bool     isOpen() const;

    // Factory & Detection
    static bool probeDevice(uint16_t vid, uint16_t pid);
    static DetectedDevice autoDetect(uint16_t vid_override, uint16_t pid_override);
    static std::unique_ptr<UsbHandler> create(const std::string &version,
                                              uint16_t vid,
                                              uint16_t pid,
                                              int timeout_ms);

protected:
    static void     packLeU32(uint8_t *dst, uint32_t val);
    static uint16_t unpackLeU16(const uint8_t *src);
    static int32_t  unpackLeI32(const uint8_t *src);
    static uint32_t rgbToU32(uint8_t r, uint8_t g, uint8_t b);
    static float    unpackLeFloat(const uint8_t *src);

    uint16_t vid_;
    uint16_t pid_;
    int      timeout_;

    libusb_context       *ctx_;
    libusb_device_handle *dev_;
    std::mutex            mutex_;
};