//
// Created by tao on 7/11/21.
//

#ifndef SRC_CONVERTER_H
#define SRC_CONVERTER_H

#include <string>
#include <fstream>
#include <ros/ros.h>

namespace kaist2bag {

std::string FilterSlash(const std::string& input);

class Converter {
public:
    Converter(const std::string& dataset_dir, const std::string& save_dir);
    virtual ~Converter() = default;

    virtual int Convert();
    virtual bool DirValid();

protected:
    void CheckAndCreateSaveDir();

protected:
    std::string dataset_dir_;
    std::string save_dir_;
    bool dir_valid_;
};


} // namespace kaist2bag

#endif //SRC_CONVERTER_H
