/*
 * Copyright 2018 The Cartographer Authors
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

#include "cartographer/mapping/internal/range_data_collator.h"

#include <memory>

#include "absl/memory/memory.h"
#include "cartographer/mapping/internal/local_slam_result_data.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

constexpr float RangeDataCollator::kDefaultIntensityValue;

sensor::TimedPointCloudOriginData RangeDataCollator::AddRangeData(
    const std::string& sensor_id,
    sensor::TimedPointCloudData timed_point_cloud_data) {
  CHECK_NE(expected_sensor_ids_.count(sensor_id), 0);
  timed_point_cloud_data.intensities.resize(
      timed_point_cloud_data.ranges.size(), kDefaultIntensityValue);
  // TODO(gaschler): These two cases can probably be one.
  /**
   * id_to_pending_data_.count(sensor_id) != 0 ,
   * 表示已经存有该雷达数据，则开始处理上一帧存的雷达数据，处理完之后，在将本帧雷达数据，存在map中。
   * ***/ 
  if (id_to_pending_data_.count(sensor_id) != 0) {
    
    LOG(ERROR)<<"\n sensor_id = "<<sensor_id;

    current_start_ = current_end_;
    // When we have two messages of the same sensor, move forward the older of
    // the two (do not send out current).
    current_end_ = id_to_pending_data_.at(sensor_id).time;
    auto result = CropAndMerge();
    id_to_pending_data_.emplace(sensor_id, std::move(timed_point_cloud_data));
    return result;
  }
  // 如果 sensor_id 键值还在,则 emplace 操作失败,返回原来的值.
  /*
      容器中不存在该雷达数据，在先把该雷达数据存入容器中，然后找其他雷达数据最早的，作为current_end_时间
  */ 
  id_to_pending_data_.emplace(sensor_id, std::move(timed_point_cloud_data));
  if (expected_sensor_ids_.size() != id_to_pending_data_.size()) {
    LOG(ERROR)<<"\n sensor_id = "<<sensor_id;
    return {};
  }
  current_start_ = current_end_;
  // We have messages from all sensors, move forward to oldest.
  common::Time oldest_timestamp = common::Time::max();
  for (const auto& pair : id_to_pending_data_) {
    oldest_timestamp = std::min(oldest_timestamp, pair.second.time);
  }
  // LOG(ERROR)
  //     <<"\n sensor_id = "<<sensor_id
  //     <<"\n expected_sensor_ids_.size() = "<<expected_sensor_ids_.size()
  //     <<"\n id_to_pending_data_.size() = "<<id_to_pending_data_.size();
  current_end_ = oldest_timestamp;
  return CropAndMerge();
}

// 同步裁剪不同的雷达数据
sensor::TimedPointCloudOriginData RangeDataCollator::CropAndMerge() {
  /*
      struct TimedPointCloudOriginData {
                                        struct RangeMeasurement {
                                          TimedRangefinderPoint point_time;
                                          float intensity;
                                          size_t origin_index; //表示雷达
                                        };
                                        common::Time time;
                                        std::vector<Eigen::Vector3f> origins;
                                        std::vector<RangeMeasurement> ranges;
                                        };
  */ 
//  以 current_end_ 作为该帧雷达数据的时间戳,后面需要将该帧所有点的时间戳,转换到相对与该时间的时间戳
  sensor::TimedPointCloudOriginData result{current_end_, {}, {}};
  bool warned_for_dropped_points = false;
  // 遍历不同的雷达传感器(支持多个雷达数据建图)
  for (auto it = id_to_pending_data_.begin();
       it != id_to_pending_data_.end();) {
    sensor::TimedPointCloudData& data = it->second;
    const sensor::TimedPointCloud& ranges = it->second.ranges;
    const std::vector<float>& intensities = it->second.intensities;

    // 找到该雷达，在起点时刻的点
    auto overlap_begin = ranges.begin();
    while (overlap_begin < ranges.end() &&
           data.time + common::FromSeconds((*overlap_begin).time) <
               current_start_) {
      ++overlap_begin;
    }

    // 找到该雷达数据在终点时刻的点
    auto overlap_end = overlap_begin;
    while (overlap_end < ranges.end() &&
           data.time + common::FromSeconds((*overlap_end).time) <=
               current_end_) {
      ++overlap_end;
    }
    if (ranges.begin() < overlap_begin && !warned_for_dropped_points) {
      LOG(WARNING) << "Dropped " << std::distance(ranges.begin(), overlap_begin)
                   << " earlier points.";
      warned_for_dropped_points = true;
    }

    // Copy overlapping range.
    if (overlap_begin < overlap_end) {
      // 找到result中点的数量，因为遍历不同的雷达，每一个雷达点，都有一个是第几个雷达的数据的标识
      std::size_t origin_index = result.origins.size();
      // 存放该帧点云数据的原点坐标（最后一个点的base坐标）
      result.origins.push_back(data.origin);
      const float time_correction =
          static_cast<float>(common::ToSeconds(data.time - current_end_));
      auto intensities_overlap_it =
          intensities.begin() + (overlap_begin - ranges.begin());

      /*
          重新定义result.ranges的大小.之前的数据量+该帧雷达的数据点云数量.
          result.ranges 存放对应在[current_start_,current_end_]时段的,所有雷达点云数据.
      */ 
      result.ranges.reserve(result.ranges.size() +
                            std::distance(overlap_begin, overlap_end));
      for (auto overlap_it = overlap_begin; overlap_it != overlap_end;
           ++overlap_it, ++intensities_overlap_it) {
        sensor::TimedPointCloudOriginData::RangeMeasurement point{
            *overlap_it, *intensities_overlap_it, origin_index};
        // current_end_ + point_time[3]_after == in_timestamp +
        // point_time[3]_before
        /**
         * point.point_time.time = (*overlap_it).time
         * point.point_time.time += time_correction => point.point_time.time = data.time + (*overlap_it).time - current_end_
         * **/ 
        // 将点时间戳转换到 相对 current_end_ 的时间戳
        point.point_time.time += time_correction;
        result.ranges.push_back(point);
      }
    }

    // Drop buffered points until overlap_end.
    // 表示该帧雷达数据使用完,可以删除该帧雷达数据
    if (overlap_end == ranges.end()) {
      it = id_to_pending_data_.erase(it);
    } else if (overlap_end == ranges.begin()) {
      ++it;
    } else {
      const auto intensities_overlap_end =
          intensities.begin() + (overlap_end - ranges.begin());
      
      // 数据裁剪,截取剩余的数据,放入到data中.因为 sensor::TimedPointCloudData& data = it->second;
      // data是原数据的引用,故直接截取赋值
      data = sensor::TimedPointCloudData{
          data.time, data.origin,
          sensor::TimedPointCloud(overlap_end, ranges.end()),
          std::vector<float>(intensities_overlap_end, intensities.end())};
      ++it;
    }
  }

  // 将所有的雷达点云数据,按时间先后顺序进行排序
  std::sort(result.ranges.begin(), result.ranges.end(),
            [](const sensor::TimedPointCloudOriginData::RangeMeasurement& a,
               const sensor::TimedPointCloudOriginData::RangeMeasurement& b) {
              return a.point_time.time < b.point_time.time;
            });
  return result;
}

}  // namespace mapping
}  // namespace cartographer
