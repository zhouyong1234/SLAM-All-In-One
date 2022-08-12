//
// Created by tao on 7/22/21.
//

#ifndef SRC_VRS_CONVERTER_H
#define SRC_VRS_CONVERTER_H

#include <string>
#include "converter.h"

namespace kaist2bag {

class VrsConverter : public Converter {
public:
    VrsConverter(const std::string& dataset_dir, const std::string& save_dir, const std::string& topic);
    virtual ~VrsConverter() = default;

    int Convert() override;

    std::string default_data_file = "sensor_data/vrs_gps.csv";

private:
    std::string topic_;
    std::string bag_name_;
};


} // namespace kaist2bag

#endif //SRC_VRS_CONVERTER_H
