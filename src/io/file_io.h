//
// Created by xiang on 25-3-12.
//

#ifndef LIGHTNING_FILE_IO_H
#define LIGHTNING_FILE_IO_H

#include "common/eigen_types.h"
#include "common/std_types.h"

namespace lightning {

/**
 * 检查某个路径是否存在
 * @param file_path 路径名
 * @return true if exist
 */
bool PathExists(const std::string& file_path);

}

#endif  // LIGHTNING_FILE_IO_H
