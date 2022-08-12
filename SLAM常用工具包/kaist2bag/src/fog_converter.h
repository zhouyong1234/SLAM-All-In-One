//
// Created by tao on 7/21/21.
//

#ifndef SRC_FOG_CONVERTER_H
#define SRC_FOG_CONVERTER_H

#include <string>
#include "converter.h"

namespace kaist2bag {

class FogConverter : public Converter {
public:
    FogConverter(const std::string& dataset_dir, const std::string& save_dir, const std::string& topic);
    virtual ~FogConverter() = default;

    int Convert() override;

    std::string default_data_file = "sensor_data/fog.csv";

private:
    std::string topic_;
    std::string bag_name_;
};


} // namespace kaist2bag

#endif //SRC_FOG_CONVERTER_H
