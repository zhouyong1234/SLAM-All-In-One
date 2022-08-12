// Copyright 2021 SuiFang

#pragma once

#include "yaml-cpp/yaml.h"
#include "object.h"
#include "matcher.h"
#include "tracker.h"
#include "preprocess.h"
#include "predictor.h"

namespace kit {
namespace perception {
namespace fusion {

class FusionWrapper {
 public:
     FusionWrapper(const std::string& config_file);

     bool Update(const FramePtr &frame);
     bool GetFusionResult(const FusionObjectListPtr &res);

 private:
    std::shared_ptr<fusion::Tracker> tracker_;
    std::shared_ptr<fusion::Matcher> matcher_;
    std::shared_ptr<fusion::Predictor> predictor_;
};

}  // namespace fusion
}  // namespace perception
}  // namespace kit
