#pragma once

#include <string>
#include <algorithm>
#include <cstring>

// Проверка расширения файла (регистронезависимо)
bool has_extension(const std::string& path, const char* ext);

// Типы медиафайлов
bool is_image(const std::string& path);
bool is_video(const std::string& path);
bool is_audio(const std::string& path);
bool is_url(const std::string& path);