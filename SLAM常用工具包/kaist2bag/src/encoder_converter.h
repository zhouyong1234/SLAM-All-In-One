//
// Created by tao on 7/21/21.
//

#ifndef SRC_ENCODER_CONVERTER_H
#define SRC_ENCODER_CONVERTER_H

#include "converter.h"

namespace kaist2bag {

class EncoderConverter : public Converter {
public:
    EncoderConverter(const std::string& dataset_dir, const std::string& save_dir, const std::string &irp_topic, const std::string &raw_topic);
    virtual ~EncoderConverter() = default;

    int Convert() override;

    std::string default_data_file = "sensor_data/encoder.csv";
    std::string default_calib_file = "calibration/EncoderParameter.txt";

private:
    std::string irp_topic_;
    std::string raw_topic_;
    std::string irp_bag_name_;
    std::string raw_bag_name_;
};




} // namespace kaist2bag

#endif //SRC_ENCODER_CONVERTER_H
