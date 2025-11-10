//
// Created by xiang on 25-3-12.
//

#include "io/file_io.h"
#include <filesystem>

namespace lightning {
bool PathExists(const std::string& file_path) {
    std::filesystem::path path(file_path);
    return std::filesystem::exists(path);
}

}  // namespace lightning