#pragma once

#include <assert.h>
#include <deque>

#include <glog/logging.h>

#include <BaseType/Measurement.h>

namespace TGK {
namespace DataSynchronizer {

inline bool InterpolateWheelData(const BaseType::WheelData& prev_wheel, 
                                 const BaseType::WheelData& next_wheel,
                                 const double inter_timestamp,
                                 BaseType::WheelData* inter_wheel) {
    assert(inter_wheel != nullptr);

    const double time_interval = next_wheel.timestamp - prev_wheel.timestamp;
    if (time_interval < 1e-6) {
        LOG(ERROR) << "[InterpolateWheelData]: Time interval between two wheel data is too small!";
        return false;
    }

    if (inter_timestamp < prev_wheel.timestamp || inter_timestamp > next_wheel.timestamp) {
        LOG(ERROR) << "[InterpolateWheelData]: The interpolate timestamp is not in the interval of the two Wheel data.";
        return false;
    }
    
    const double ratio = (inter_timestamp - prev_wheel.timestamp) / time_interval;
    const double one_minus_ratio = 1. - ratio;
    inter_wheel->timestamp = inter_timestamp;
    inter_wheel->left = ratio * next_wheel.left + one_minus_ratio * prev_wheel.left;
    inter_wheel->right = ratio * next_wheel.right + one_minus_ratio * prev_wheel.right;

    return true;
}

inline bool CollectWheelDataBetweenTimes(const std::deque<BaseType::WheelDataConstPtr>& wheel_buffer, 
                                       const double start_time, const double end_time, 
                                       std::vector<BaseType::WheelDataConstPtr>* wheels) {
    assert(wheels != nullptr);

    if (wheel_buffer.empty()) {
        LOG(ERROR) << "[CollectWheelDataBetweenTimes]: Wheel buffer is empty!";
        return false;
    }

    if (start_time < wheel_buffer.front()->timestamp || end_time > wheel_buffer.back()->timestamp) {
        LOG(ERROR) << "[CollectWheelDataBetweenTimes]: Search timestamp is out of the Wheel buffer!";
        return false;
    }

    // Check timestamp.
    const double time_interval = end_time - start_time;
    // TODO: make it to be configs.
    constexpr double kMinWheelTimeInterval = 0.0001;
    if (time_interval < kMinWheelTimeInterval) {
        LOG(ERROR) << "[CollectWheelDataBetweenTimes]: The interval between start and end time is too small!";
        return false;
    }

    // Get end_idx. 
    int end_idx = wheel_buffer.size() - 1;
    while (wheel_buffer[end_idx]->timestamp >= end_time) {
        --end_idx;
    }

    // Get start idx.
    int start_idx = end_idx;
    while (wheel_buffer[start_idx]->timestamp > start_time) {
        --start_idx;
    }

    // Interpolate the first data.
    BaseType::WheelData start_wheel;
    if (!InterpolateWheelData(*(wheel_buffer[start_idx]), *(wheel_buffer[start_idx + 1]), start_time, &start_wheel)) {
        LOG(ERROR) << "[CollectWheelDataBetweenTimes]: Failed to interpolate the start Wheel data!";
        return false;
    }
    wheels->push_back(std::make_shared<BaseType::WheelData>(start_wheel));

    for (size_t i = start_idx + 1; i <= end_idx; ++i) {
        wheels->push_back(wheel_buffer[i]);
    }

    BaseType::WheelData end_wheel;
    if (!InterpolateWheelData(*(wheel_buffer[end_idx]), *(wheel_buffer[end_idx + 1]), end_time, &end_wheel)) {
        LOG(ERROR) << "[CollectWheelDataBetweenTimes]: Failed to interplate the end Wheel data!";
    }
    wheels->push_back(std::make_shared<BaseType::WheelData>(end_wheel));

    return true;
}

}  // namespace DataSynchronizer
}  // namespace TGK