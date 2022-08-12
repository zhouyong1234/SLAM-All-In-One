//
// Created by tao on 7/11/21.
//

#ifndef SRC_ALTIMETER_CONVERTER_H
#define SRC_ALTIMETER_CONVERTER_H

#include <string>
#include "converter.h"

namespace kaist2bag {

class AltimeterConverter : public Converter {
public:
    AltimeterConverter(const std::string& dataset_dir, const std::string& save_dir, const std::string& topic);
    virtual ~AltimeterConverter() = default;

    int Convert() override;

    std::string default_data_file = "sensor_data/altimeter.csv";

private:
    std::string topic_;
    std::string bag_name_;
};


} // namespace kaist2bag

#endif //SRC_ALTIMETER_CONVERTER_H
