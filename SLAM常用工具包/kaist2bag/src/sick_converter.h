//
// Created by tao on 7/28/21.
//

#ifndef SRC_SICK_CONVERTER_H
#define SRC_SICK_CONVERTER_H
#include <string>
#include "converter.h"

namespace kaist2bag {

class SickConverter : public Converter {
public:
    SickConverter(const std::string& dataset_dir, const std::string& save_dir,
                      const std::string& back_topic, const std::string& middle_topic);
    virtual ~SickConverter() = default;

    int Convert() override;

    std::string default_back_stamp_file = "sensor_data/SICK_back_stamp.csv";
    std::string default_back_data_dir = "sensor_data/SICK_back";
    std::string default_middle_stamp_file = "sensor_data/SICK_middle_stamp.csv";
    std::string default_middle_data_dir = "sensor_data/SICK_middle";

private:
    std::string back_topic_;
    std::string back_bag_name_;
    std::string middle_topic_;
    std::string middle_bag_name_;

    void Convert(const std::string& stamp_file, const std::string& data_dir,
                 const std::string& bag_file, const std::string& topic,
                 const std::string& frame_id);
};


} // namespace kaist2bag
#endif //SRC_SICK_CONVERTER_H
