//
// Created by tao on 7/11/21.
//

#include "converter.h"

#include <sys/stat.h>
#include <algorithm>
#include <boost/filesystem.hpp>

namespace kaist2bag {

std::string FilterSlash(const std::string& input) {
    if (input.empty()) return std::string();
    std::string str = input;
    if (*str.begin() == '/') {
        str.erase(str.begin());
        if (str.empty()) return std::string();
    }

    if (*(str.end() - 1) == '/') {
        str.erase(str.end() - 1);
        if (str.empty()) return std::string();
    }
    std::replace(str.begin(), str.end(), '/', '_');
    return str;
}

Converter::Converter(const std::string &dataset_dir, const std::string &save_dir)
    : dataset_dir_(dataset_dir), save_dir_(save_dir){
    struct stat buffer;
    dir_valid_ = (stat(dataset_dir_.c_str(), &buffer) == 0);
}

int Converter::Convert() {
    return 0;
}

bool Converter::DirValid() {
    return dir_valid_;
}

void Converter::CheckAndCreateSaveDir() {
    if (!boost::filesystem::is_directory(save_dir_) || !boost::filesystem::exists(save_dir_)) {
        boost::filesystem::create_directory(save_dir_);
    }
}

} // namespace kaist2bag