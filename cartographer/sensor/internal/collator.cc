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

#include "cartographer/sensor/internal/collator.h"

namespace cartographer {
namespace sensor {

// 将所有数据信息,都放入队列里,防止机器人处理传感器数据不及时,导致出现漏帧问题.
void Collator::AddTrajectory(
    const int trajectory_id,
    const absl::flat_hash_set<std::string>& expected_sensor_ids,
    const Callback& callback) {                                 //callback = HandleCollatedSensorData
  for (const auto& sensor_id : expected_sensor_ids) {
    const auto queue_key = QueueKey{trajectory_id, sensor_id};
    
    // 初始化不同传感器的数据队列queue，并传入不同传感器的数据处理方法，来处理数据
    queue_.AddQueue(queue_key,
                    [callback, sensor_id](std::unique_ptr<Data> data) {
                      callback(sensor_id, std::move(data));
                    });
    queue_keys_[trajectory_id].push_back(queue_key);
  }
}

void Collator::FinishTrajectory(const int trajectory_id) {
  for (const auto& queue_key : queue_keys_[trajectory_id]) {
    queue_.MarkQueueAsFinished(queue_key);
  }
}

// 向传感器数据队列中，添加传感器数据
void Collator::AddSensorData(const int trajectory_id,
                             std::unique_ptr<Data> data) {
  // trajectory_id 和 sendor_id 作为键值
  QueueKey queue_key{trajectory_id, data->GetSensorId()};
  queue_.Add(std::move(queue_key), std::move(data));
}

void Collator::Flush() { queue_.Flush(); }

absl::optional<int> Collator::GetBlockingTrajectoryId() const {
  return absl::optional<int>(queue_.GetBlocker().trajectory_id);
}

}  // namespace sensor
}  // namespace cartographer
