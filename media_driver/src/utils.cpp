#include "media_driver/utils.hpp"

// Универсальная вспомогательная функция для проверки расширения
bool has_extension(const std::string& path, const char* ext) {
  // Защита от некорректных входных данных
  if (!ext || *ext == '\0') return false;
  
  const size_t ext_len = std::strlen(ext);
  const size_t path_len = path.size();
  
  // Проверка длины: расширение не может быть длиннее пути
  if (path_len < ext_len) return false;
  
  // Сравниваем последние символы БЕЗ изменения указателей в цикле
  for (size_t i = 0; i < ext_len; ++i) {
    // path[path_len - ext_len + i] — i-й символ расширения в пути
    // ext[i] — i-й символ эталонного расширения
    if (std::tolower(static_cast<unsigned char>(path[path_len - ext_len + i])) != 
        std::tolower(static_cast<unsigned char>(ext[i]))) {
      return false;
    }
  }
  return true;
}


// Проверка изображений
bool is_image(const std::string& path) {
  return has_extension(path, ".png")  || has_extension(path, ".jpg")   || 
         has_extension(path, ".jpeg") || has_extension(path, ".bmp")   || 
         has_extension(path, ".webp") || has_extension(path, ".gif")   || 
         has_extension(path, ".tiff") || has_extension(path, ".tif")   || 
         has_extension(path, ".svg");
}

// Проверка видео
bool is_video(const std::string& path) {
  return has_extension(path, ".mp4")  || has_extension(path, ".mov")   || 
         has_extension(path, ".avi")  || has_extension(path, ".mkv")   || 
         has_extension(path, ".webm") || has_extension(path, ".flv")   || 
         has_extension(path, ".wmv")  || has_extension(path, ".m4v")   || 
         has_extension(path, ".mpeg") || has_extension(path, ".mpg")   || 
         has_extension(path, ".3gp")  || has_extension(path, ".ts")    || 
         has_extension(path, ".m3u8") || has_extension(path, ".m3u");
}

// Проверка аудио
bool is_audio(const std::string& path) {
  return has_extension(path, ".mp3")  || has_extension(path, ".wav")   || 
         has_extension(path, ".ogg")  || has_extension(path, ".flac")  || 
         has_extension(path, ".aac")  || has_extension(path, ".m4a")   || 
         has_extension(path, ".wma")  || has_extension(path, ".opus")  || 
         has_extension(path, ".aiff") || has_extension(path, ".aif");
}
