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
// 将value 转换为 概率
float SlowValueToBoundedFloat(const uint16 value, const uint16 unknown_value,
                              const float unknown_result,
                              const float lower_bound,
                              const float upper_bound) {
  CHECK_LT(value, kValueCount);
  if (value == unknown_value) return unknown_result;
  const float kScale = (upper_bound - lower_bound) / (kValueCount - 2.f);

  // (value - 1) * kScale + lower_bound
  return value * kScale + (lower_bound - kScale);
}

/**
 * unknown_value = 0
 * unknown_result = 0.9
 * lower_bound = 0.1
 * upper_bound = 0.9
 * 0 is unknown_result=0.9, [1, 32767] maps to [lower_bound, upper_bound].
 * **/ 
std::unique_ptr<std::vector<float>> PrecomputeValueToBoundedFloat(
    const uint16 unknown_value, const float unknown_result,
    const float lower_bound, const float upper_bound) {
  auto result = absl::make_unique<std::vector<float>>();
  // Repeat two times, so that both values with and without the update marker
  // can be converted to a probability.
  constexpr int kRepetitionCount = 2;
  // kRepetitionCount * kValueCount = 2 * 32768
  result->reserve(kRepetitionCount * kValueCount);
  for (int repeat = 0; repeat != kRepetitionCount; ++repeat) {
    for (int value = 0; value != kValueCount; ++value) {
      result->push_back(SlowValueToBoundedFloat(
          value, unknown_value, unknown_result, lower_bound, upper_bound));
    }
  }
  return result;
}

// 表:以value为索引,probability 为值
std::unique_ptr<std::vector<float>> PrecomputeValueToProbability() {
  return PrecomputeValueToBoundedFloat(kUnknownProbabilityValue,
                                       kMinProbability, kMinProbability,
                                       kMaxProbability);
}

/**
 * kUnknownCorrespondenceValue = 0
 * kMaxCorrespondenceCost = 0.9
 * kMinCorrespondenceCost = 0.1
 * **/ 
std::unique_ptr<std::vector<float>> PrecomputeValueToCorrespondenceCost() {
  return PrecomputeValueToBoundedFloat(
      kUnknownCorrespondenceValue, kMaxCorrespondenceCost,
      kMinCorrespondenceCost, kMaxCorrespondenceCost);
}

}  // namespace

// 占据value 转换到 概率的表
const std::vector<float>* const kValueToProbability =
    PrecomputeValueToProbability().release();

// 空闲value 转换到 概率的表
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

/**
 * odds = 0.55/(1-0.55) 传感器概率
 * kValueCount = 32768
 * ProbabilityFromOdds(odds) {return odds/(1 + odds)}
 * ProbabilityToCorrespondenceCost(p) {return 1-p}
 * CorrespondenceCostToValue(value) 将value转换到[1,32768]
 * 返回 hit_table_
 *    hit_table_[i] 表示 value = i 时,再次观测到该点之后的 value.
 *    即: 
 *        *kValueToCorrespondenceCost)[cell] 该点原来的概率(value = cell时,对应概率)
 *        Odds(CorrespondenceCostToProbability((*kValueToCorrespondenceCost)[cell]) 计算原来的 odd(s)
 *        odds 雷达观测到该点状态, odds = 0.55/(1-0.55)
 *        odds * Odds(CorrespondenceCostToProbability((*kValueToCorrespondenceCost)[cell])) 更新概率 odd(s|z) = odds*odd(s)
 *  
 * 
 * kValueToCorrespondenceCost[2*32768]
 * ***/ 
std::vector<uint16> ComputeLookupTableToApplyCorrespondenceCostOdds(
    float odds) {
  std::vector<uint16> result;
  result.reserve(kValueCount);
  /*
    hit:
      ProbabilityFromOdds(odds) = 0.55      计算栅格被占据的概率
      ProbabilityToCorrespondenceCost(ProbabilityFromOdds(odds)) = 1-0.55=0.45 空闲概率
      CorrespondenceCostToValue(0.45) = (0.45 - 0.1)*32766/(0.9-0.1)+1
      hit_table_[0] = (0.45 - 0.1)*32766/(0.9-0.1) + 1 + kUpdateMarker.上一次概率为0,被hit之后的概率为 hit_table_[0]
  */ 
  result.push_back(CorrespondenceCostToValue(ProbabilityToCorrespondenceCost(
                       ProbabilityFromOdds(odds))) +
                   kUpdateMarker);
  for (int cell = 1; cell != kValueCount; ++cell) {
    /*
        p = (*kValueToCorrespondenceCost)[cell] ==> 将 cell(空闲概率) 映射到 [0.1,0.9] 概率范围内
        p' = CorrespondenceCostToProbability(p) = 1-p  占据概率
        Odds(p') = p'/(1-p') = (1-p)/p
        概率更新: new_odds = odds * Odds(p')
        计算概率: new_p = ProbabilityFromOdds(new_odds)  更新后占据概率
        new_p' = ProbabilityToCorrespondenceCost(new_p) = 1-new_p   更新后空闲概率
        value = CorrespondenceCostToValue(new_p') = (new_p' - 0.1)*32766/(0.9-0.1)+1
        result[cell] = value + kUpdateMarker;
    */ 
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
