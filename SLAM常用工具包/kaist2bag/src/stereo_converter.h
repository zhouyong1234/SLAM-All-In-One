//
// Created by tao on 7/28/21.
//

#ifndef SRC_STEREO_CONVERTER_H
#define SRC_STEREO_CONVERTER_H
#include <string>
#include "converter.h"

namespace kaist2bag {

class StereoConverter : public Converter {
public:
    StereoConverter(const std::string& dataset_dir, const std::string& save_dir,
                      const std::string& left_topic, const std::string& right_topic);
    virtual ~StereoConverter() = default;

    int Convert() override;

    std::string default_stamp_file = "sensor_data/stereo_stamp.csv";
    std::string default_left_data_dir = "image/stereo_left";
    std::string default_right_data_dir = "image/stereo_right";

private:
    std::string left_topic_;
    std::string left_bag_name_;
    std::string right_topic_;
    std::string right_bag_name_;

    void Convert(const std::string& stamp_file, const std::string& data_dir,
                 const std::string& bag_file, const std::string& topic,
                 const std::string& frame_id);
};


} // namespace kaist2bag
#endif //SRC_STEREO_CONVERTER_H
