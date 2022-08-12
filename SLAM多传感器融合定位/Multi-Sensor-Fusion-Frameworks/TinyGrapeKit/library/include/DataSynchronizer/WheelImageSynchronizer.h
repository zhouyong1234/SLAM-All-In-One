#pragma once

#include <BaseType/Measurement.h>

namespace TGK {
namespace DataSynchronizer {

class WheelImageSynchronizer {
public:
    WheelImageSynchronizer(const int max_wheel_buffer_length = 1000, 
                           const int max_image_buffer_length = 10);

    bool FeedWheelData(const BaseType::WheelDataConstPtr& wheel_data, 
                       std::vector<BaseType::WheelDataConstPtr>* wheel_data_vec,
                       BaseType::MonoImageDataConstPtr* mono_image_data);
    
    void FeedMonoImageData(const BaseType::MonoImageDataConstPtr& mono_image_data);

private:
    int max_wheel_buffer_length_;
    int max_image_buffer_length_;
    double last_image_timestamp_;

    std::deque<BaseType::WheelDataConstPtr> wheel_buffer_;
    std::deque<BaseType::MonoImageDataConstPtr> image_buffer_;
};

}  // namespace DataSynchronizer
}  // namespace TGK