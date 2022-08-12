//
// Created by tao on 7/22/21.
//

#ifndef SRC_GPS_CONVERTER_H
#define SRC_GPS_CONVERTER_H

#include <string>
#include "converter.h"

namespace kaist2bag {

class GpsConverter : public Converter {
public:
    GpsConverter(const std::string& dataset_dir, const std::string& save_dir, const std::string& topic);
    virtual ~GpsConverter() = default;

    int Convert() override;

    std::string default_data_file = "sensor_data/gps.csv";

private:
    std::string topic_;
    std::string bag_name_;
};


} // namespace kaist2bag

#endif //SRC_GPS_CONVERTER_H
