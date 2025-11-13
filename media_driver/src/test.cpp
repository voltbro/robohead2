

// g++ -std=c++17 test.cpp -o player -lmpv

// mpv_player.cpp
#include <stdexcept>
#include <iostream>
#include <thread>

// mpv_player.hpp
#pragma once
#include <mpv/client.h>
#include <string>

class MpvPlayer {
public:
    MpvPlayer();
    ~MpvPlayer();

    // Показать изображение или видео
    void play(const std::string& video_path, const std::string& audio_path = "");

private:
    mpv_handle* mpv_ = nullptr;
};

MpvPlayer::MpvPlayer() {
    mpv_ = mpv_create();
    if (!mpv_) throw std::runtime_error("mpv_create failed");

    // Настройки
    mpv_set_option_string(mpv_, "vo", "drm"); // or gpu
    // mpv_set_option_string(mpv_, "vo", "gpu");
    // mpv_set_option_string(mpv_, "gpu-context", "drm");
    mpv_set_option_string(mpv_, "ao", "alsa");        // или "pcm", "null"
    mpv_set_option_string(mpv_, "hwdec", "auto");
    mpv_set_option_string(mpv_, "keep-open", "yes");  // Не закрывать после конца
    // mpv_set_option_string(mpv_, "keep-open-pause", "yes");  // Не закрывать после конца
    mpv_set_option_string(mpv_, "image-display-duration", "86400"); // 24 часа — "навсегда"
    mpv_set_option_string(mpv_, "loop-file", "no");
    mpv_set_option_string(mpv_, "idle", "yes");       // Разрешить пустой старт
    mpv_set_option_string(mpv_, "force-window", "yes");
    mpv_set_option_string(mpv_, "background", "black");
    mpv_request_log_messages(mpv_, "debug");

    if (mpv_initialize(mpv_) < 0) {
        mpv_terminate_destroy(mpv_);
        mpv_ = nullptr;
        throw std::runtime_error("mpv_initialize failed");
    }
}

MpvPlayer::~MpvPlayer() {
    if (mpv_) mpv_terminate_destroy(mpv_);
}



void MpvPlayer::play(const std::string& video_path, const std::string& audio_path) {
    if (!mpv_) return;
    const char* stop_cmd[] = {"stop", nullptr};
    mpv_command(mpv_, stop_cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    const char* cmd[] = {"loadfile", video_path.c_str(), "replace", nullptr};
    mpv_command(mpv_, cmd);
    if (!audio_path.empty()) {
        const char* cmd_audio[] = {"audio-add", audio_path.c_str(), nullptr};
        mpv_command(mpv_, cmd_audio);
        mpv_set_property_string(mpv_, "aid", "auto");
    }

    // Принудительно активировать видео и снять паузу
    // mpv_set_property_string(mpv_, "vid", "1");
    // mpv_set_property_string(mpv_, "aid", "1");
    mpv_set_property_string(mpv_, "pause", "no");
}



#include <chrono>

int main() {
    freopen("/tmp/mpv.log", "w", stderr);
    MpvPlayer player;
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    player.play("/home/pi/robohead_ws/src/robohead2/robohead_controller/actions/std_attention/attention.png", "/home/pi/robohead_ws/src/robohead2/robohead_controller/actions/std_attention/attention.mp3");
    // player.play("/home/pi/robohead_ws/src/robohead2/robohead_controller/actions/std_attention/attention.png");
  
    std::this_thread::sleep_for(std::chrono::seconds(2));

    player.play("/home/pi/robohead_ws/src/robohead2/robohead_controller/actions/std_wait/wait.png"); // с родным звуком
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // player.play("/home/pi/robohead_ws/src/robohead2/robohead_controller/actions/std_greeting/greeting.png", "/home/pi/robohead_ws/src/robohead2/robohead_controller/actions/std_greeting/greeting.mp3"); // картинка + звук
    player.play("/home/pi/loadingVideo.mp4"); // картинка + звук
    std::this_thread::sleep_for(std::chrono::seconds(3));

    player.play("/home/pi/video_audio.mov", "/home/pi/file.mp3"); // картинка + звук
    std::this_thread::sleep_for(std::chrono::seconds(6));



    player.play("/home/pi/robohead_ws/src/robohead2/robohead_controller/actions/std_attention/attention.png"); // с родным звуком
    std::this_thread::sleep_for(std::chrono::seconds(3));

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    return 0;
}