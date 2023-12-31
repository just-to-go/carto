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

#ifndef CARTOGRAPHER_SENSOR_INTERNAL_DISPATCHABLE_H_
#define CARTOGRAPHER_SENSOR_INTERNAL_DISPATCHABLE_H_

#include "cartographer/mapping/trajectory_builder_interface.h"
#include "cartographer/sensor/data.h"

namespace cartographer {
namespace sensor {

template <typename DataType>
class Dispatchable : public Data {
 public:
  Dispatchable(const std::string &sensor_id, const DataType &data)
      : Data(sensor_id), data_(data) {}

  common::Time GetTime() const override { return data_.time; }

  // 数据　DataType 中，封装的处理数据方法　trajectory_builder　= global_trajectory_builder
  // 调用 trajectory_builder 中方法,处理传感器数据
  void AddToTrajectoryBuilder(
      mapping::TrajectoryBuilderInterface *const trajectory_builder) override {
    trajectory_builder->AddSensorData(sensor_id_, data_);
  }
  const DataType &data() const { return data_; }

 private:
  const DataType data_;
};

// collated_trajectory_builder.h 中调用
// 将数据封装成　Dispatchable 数据，Dispatchable中封装了　AddToTrajectoryBuilder() 函数来处理传感器数据
template <typename DataType>
std::unique_ptr<Dispatchable<DataType>> MakeDispatchable(
    const std::string &sensor_id, const DataType &data) {
  return absl::make_unique<Dispatchable<DataType>>(sensor_id, data);
}

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_INTERNAL_DISPATCHABLE_H_
