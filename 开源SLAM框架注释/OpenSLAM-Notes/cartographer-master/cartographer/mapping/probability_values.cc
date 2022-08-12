/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/mapping/probability_values.h"

#include "absl/memory/memory.h"

namespace cartographer {
namespace mapping {

namespace {

constexpr int kValueCount = 32768;

// 0 is unknown, [1, 32767] maps to [lower_bound, upper_bound].
// 将 [1,32767]范围映射到 lower_bound, upper_bound
float SlowValueToBoundedFloat(const uint16 value, const uint16 unknown_value,
                              const float unknown_result,
                              const float lower_bound,
                              const float upper_bound) {
  CHECK_LT(value, kValueCount);
  if (value == unknown_value) return unknown_result;
  const float kScale = (upper_bound - lower_bound) / (kValueCount - 2.f);
  return value * kScale + (lower_bound - kScale);
}

// 生成给定范围内的浮点数组
// unknown_value   
// unknown_result  
// lower_bound     范围下界
// upper_bound     范围上界    
std::unique_ptr<std::vector<float>> PrecomputeValueToBoundedFloat(
    const uint16 unknown_value, const float unknown_result,
    const float lower_bound, const float upper_bound) {
  // 浮点数组  
  auto result = absl::make_unique<std::vector<float>>();
  // Repeat two times, so that both values with and without the update marker
  // can be converted to a probability.
  constexpr int kRepetitionCount = 2;// 重复数
  // 分配 2*kValueCount 内存，为何两倍？
  result->reserve(kRepetitionCount * kValueCount);
  for (int repeat = 0; repeat != kRepetitionCount; ++repeat) {
    for (int value = 0; value != kValueCount; ++value) {
	  // 调用 SlowValueToBoundedFloat 函数计算
      result->push_back(SlowValueToBoundedFloat(
          value, unknown_value, unknown_result, lower_bound, upper_bound));
    }
  }
  return result;
}

std::unique_ptr<std::vector<float>> PrecomputeValueToProbability() {
  return PrecomputeValueToBoundedFloat(kUnknownProbabilityValue,
                                       kMinProbability, kMinProbability,
                                       kMaxProbability);
}
// 生成变量 kValueToCorrespondenceCost
// 调用函数 PrecomputeValueToBoundedFloat
// cost [0.1,0.9] --> [1,32768] 的一个映射
std::unique_ptr<std::vector<float>> PrecomputeValueToCorrespondenceCost() {
  return PrecomputeValueToBoundedFloat(
      kUnknownCorrespondenceValue, kMaxCorrespondenceCost,
      kMinCorrespondenceCost, kMaxCorrespondenceCost);
}

}  // namespace

const std::vector<float>* const kValueToProbability =
    PrecomputeValueToProbability().release();

const std::vector<float>* const kValueToCorrespondenceCost =
    PrecomputeValueToCorrespondenceCost().release();

std::vector<uint16> ComputeLookupTableToApplyOdds(const float odds) {
  std::vector<uint16> result;
  result.reserve(kValueCount);
  result.push_back(ProbabilityToValue(ProbabilityFromOdds(odds)) +
                   kUpdateMarker);
  for (int cell = 1; cell != kValueCount; ++cell) {
    result.push_back(ProbabilityToValue(ProbabilityFromOdds(
                         odds * Odds((*kValueToProbability)[cell]))) +
                     kUpdateMarker);
  }
  return result;
}
// 在概率栅格地图插入类的构造函数中被调用，用于生成hit_table_ 和 miss_table_
// odds 概率改变量
std::vector<uint16> ComputeLookupTableToApplyCorrespondenceCostOdds(
    float odds) {
  std::vector<uint16> result;
  result.reserve(kValueCount);
  result.push_back(CorrespondenceCostToValue(ProbabilityToCorrespondenceCost(
                       ProbabilityFromOdds(odds))) +
                   kUpdateMarker);
  // 遍历 kValueCount
  for (int cell = 1; cell != kValueCount; ++cell) {
    result.push_back(
        CorrespondenceCostToValue(
            ProbabilityToCorrespondenceCost(ProbabilityFromOdds(
                odds * Odds(CorrespondenceCostToProbability(
                           (*kValueToCorrespondenceCost)[cell]))))) +
        kUpdateMarker);
  }
  return result;
}

}  // namespace mapping
}  // namespace cartographer
