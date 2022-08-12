//
// Created by tao on 7/24/21.
//

#ifndef SRC_IMU_CONVERTER_H
#define SRC_IMU_CONVERTER_H
#include <string>
#include "converter.h"

namespace kaist2bag {

class ImuConverter : public Converter {
public:
    // ImuConverter(const std::string& dataset_dir, const std::string& save_dir,
    //              const std::string& irp_topic, const std::string& raw_topic,
    //              const std::string& mag_topic);

    ImuConverter(const std::string& dataset_dir, const std::string& save_dir,
                 const std::string& raw_topic);
    
    virtual ~ImuConverter() = default;

    int Convert() override;

    std::string default_data_file = "sensor_data/xsens_imu.csv";

private:
    // std::string irp_topic_;
    std::string raw_topic_;
    // std::string mag_topic_;
    // std::string irp_bag_name_;
    std::string raw_bag_name_;
    // std::string mag_bag_name_;
};


} // namespace kaist2bag
#endif //SRC_IMU_CONVERTER_H
