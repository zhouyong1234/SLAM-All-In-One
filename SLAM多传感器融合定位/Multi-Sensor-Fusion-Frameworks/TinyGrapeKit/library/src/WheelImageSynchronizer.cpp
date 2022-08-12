#include "DataSynchronizer/WheelImageSynchronizer.h"

#include <glog/logging.h>

#include <DataSynchronizer/DataSynchronizerUtil.h>

namespace TGK {
namespace DataSynchronizer {

WheelImageSynchronizer::WheelImageSynchronizer(const int max_wheel_buffer_length, 
                                               const int max_image_buffer_length)
    : max_wheel_buffer_length_(max_wheel_buffer_length), 
      max_image_buffer_length_(max_image_buffer_length),
      last_image_timestamp_(-1.) { }

bool WheelImageSynchronizer::FeedWheelData(const BaseType::WheelDataConstPtr& wheel_data, 
                                           std::vector<BaseType::WheelDataConstPtr>* wheel_data_segment,
                                           BaseType::MonoImageDataConstPtr* mono_image_data) {
    assert(wheel_data != nullptr);
    assert(wheel_data_segment != nullptr);
    assert(mono_image_data != nullptr);

    // Reject rolling back wheel data.
    if (!wheel_buffer_.empty() && wheel_data->timestamp <= wheel_buffer_.back()->timestamp) {
        LOG(ERROR) << "[FeedWheelData]: Wheel timestamp rolling back!";
        return false;
    }

    // First push data to deque.
    wheel_buffer_.push_back(wheel_data);
    if (wheel_buffer_.size() > max_wheel_buffer_length_) {
        wheel_buffer_.pop_front();
    }

    if (image_buffer_.empty()) {
        return false;
    }

    // Then check if the wheel data timestamp is bigger than the image timestamp.
    const double image_timestamp = image_buffer_.front()->timestamp;
    if (wheel_data->timestamp < image_timestamp) {
        return false;
    }

    *mono_image_data = image_buffer_.front();
    image_buffer_.pop_front();

    // Collect all wheel data before the first image.
    const double last_used_wheel_timestamp = (last_image_timestamp_ > 0.) ? 
                                             last_image_timestamp_ : 
                                             wheel_buffer_.front()->timestamp; 

    // Collect Wheel data between the last timestamp and the current image time.
    if (!CollectWheelDataBetweenTimes(wheel_buffer_, last_used_wheel_timestamp, 
                                      image_timestamp, wheel_data_segment)) {
        LOG(ERROR) << "[FeedWheelData]: Failed to collect Wheel data between times!";
        return false;
    }

    last_image_timestamp_ = image_timestamp;

    return true;
}
    
void WheelImageSynchronizer::FeedMonoImageData(const BaseType::MonoImageDataConstPtr& image_data) {
    assert(image_data != nullptr);
    
    // Reject rolling back camera data.
    if (!image_buffer_.empty() && image_data->timestamp <= image_buffer_.back()->timestamp) {
        LOG(ERROR) << "[FeedMonoImageData]: Image timestamp rolling back!";
        return;
    }

    image_buffer_.push_back(image_data);
    while (image_buffer_.size() > max_image_buffer_length_ || 
           (!wheel_buffer_.empty() && image_buffer_.front()->timestamp < wheel_buffer_.front()->timestamp)) {
        image_buffer_.pop_front();
    }
}

}  // namespace DataSynchronizer
}  // namespace TGK