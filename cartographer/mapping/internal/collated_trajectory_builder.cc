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

#include "cartographer/mapping/internal/collated_trajectory_builder.h"

#include "cartographer/common/time.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

namespace {

constexpr double kSensorDataRatesLoggingPeriodSeconds = 15.;

}  // namespace

// wrapped_trajectory_builder = global_trajectory_builder
CollatedTrajectoryBuilder::CollatedTrajectoryBuilder(
    const proto::TrajectoryBuilderOptions& trajectory_options,
    sensor::CollatorInterface* const sensor_collator, const int trajectory_id,
    const std::set<SensorId>& expected_sensor_ids,
    std::unique_ptr<TrajectoryBuilderInterface> wrapped_trajectory_builder)
    : sensor_collator_(sensor_collator),
      collate_landmarks_(trajectory_options.collate_landmarks()),
      collate_fixed_frame_(trajectory_options.collate_fixed_frame()),
      trajectory_id_(trajectory_id),
      wrapped_trajectory_builder_(std::move(wrapped_trajectory_builder)),
      last_logging_time_(std::chrono::steady_clock::now()) {
  absl::flat_hash_set<std::string> expected_sensor_id_strings;
  for (const auto& sensor_id : expected_sensor_ids) {
    if (sensor_id.type == SensorId::SensorType::LANDMARK &&
        !collate_landmarks_) {
      continue;
    }
    if (sensor_id.type == SensorId::SensorType::FIXED_FRAME_POSE &&
        !collate_fixed_frame_) {
      continue;
    }
    expected_sensor_id_strings.insert(sensor_id.id);
  }

  // 将传感器类型　和　对应的处理方法，一起封装到queue_ 中.
  // 传感器数据压入到　queue_ 中，直接使用其对应的处理函数　HandleCollatedSensorData　来处理数据．
  sensor_collator_->AddTrajectory(
      trajectory_id, expected_sensor_id_strings,
      [this](const std::string& sensor_id, std::unique_ptr<sensor::Data> data) {
        HandleCollatedSensorData(sensor_id, std::move(data));
      });
}

// sensor::Data 为　Dispatchable 数据，里面封装了该数据的处理方式
void CollatedTrajectoryBuilder::AddData(std::unique_ptr<sensor::Data> data) {
  sensor_collator_->AddSensorData(trajectory_id_, std::move(data));
}

// 传感器数据队列queue中，封装的处理数据方法
// std::map<std::string, common::RateTimer<>> 
void CollatedTrajectoryBuilder::HandleCollatedSensorData(
    const std::string& sensor_id, std::unique_ptr<sensor::Data> data) {
  // 第一次找不到对应传感器
  auto it = rate_timers_.find(sensor_id);
  if (it == rate_timers_.end()) {
    // 找不到对应传感器，初始化　rate_timers_
    it = rate_timers_
             .emplace(
                 std::piecewise_construct, std::forward_as_tuple(sensor_id),
                 std::forward_as_tuple(
                     common::FromSeconds(kSensorDataRatesLoggingPeriodSeconds)))
             .first;
    LOG(WARNING)
      <<"\n rate_timers_:"
      <<"\n kSensorDataRatesLoggingPeriodSeconds = "<<kSensorDataRatesLoggingPeriodSeconds;
  }
  it->second.Pulse(data->GetTime());

// 打印计算传感器
  if (std::chrono::steady_clock::now() - last_logging_time_ >
      common::FromSeconds(kSensorDataRatesLoggingPeriodSeconds)) {
    for (const auto& pair : rate_timers_) {
      LOG(INFO) << pair.first << " rate: " << pair.second.DebugString();
    }
    last_logging_time_ = std::chrono::steady_clock::now();
  }

  // 通过全局轨迹构建　处理传感器数据
  // 调用　Dispatchable　结构体中的操作函数
  data->AddToTrajectoryBuilder(wrapped_trajectory_builder_.get());  // dispatchable.h
}

}  // namespace mapping
}  // namespace cartographer
