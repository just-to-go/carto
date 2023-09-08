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

#include "cartographer/sensor/internal/voxel_filter.h"

#include <cmath>
#include <random>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "cartographer/common/math.h"

namespace cartographer {
namespace sensor {

namespace {

// 雷达最远距离筛选
PointCloud FilterByMaxRange(const PointCloud& point_cloud,
                            const float max_range) {
  return point_cloud.copy_if([max_range](const RangefinderPoint& point) {
    return point.position.norm() <= max_range;
  });
}

//对获取点云数据进行自适应体素滤波处理
// 尽量保证滤波之后的点云数据刚好　大于最小点云数量，
PointCloud AdaptivelyVoxelFiltered(
    const proto::AdaptiveVoxelFilterOptions& options,
    const PointCloud& point_cloud) {
  if (point_cloud.size() <= options.min_num_points()) {
    // 'point_cloud' is already sparse enough.
    LOG(WARNING)
      <<"\n 点云数据不够，直接返回："
      <<"\n point_cloud.size() = "<<point_cloud.size()
      <<"\n options.min_num_points() = "<<options.min_num_points();
    return point_cloud;
  }
  // 使用　最大距离再次进行滤波

  static bool flag = false;
  if(flag == false)
  {
    flag = true;
    LOG(WARNING)
    <<"\n VoxelFilter param:"
    <<"\n options.max_length() = "<<options.max_length();
  }

  // 以　options.max_length()　为分辨率来进行滤波计算．　以最大尺寸划分栅格来进行滤波
  PointCloud result = VoxelFilter(point_cloud, options.max_length());
  if (result.size() >= options.min_num_points()) {
    // Filtering with 'max_length' resulted in a sufficiently dense point cloud.
    //最大尺寸滤波结果,仍然满足 大于 min_num_points,直接返回result
    return result;
  }
  // Search for a 'low_length' that is known to result in a sufficiently
  // dense point cloud. We give up and use the full 'point_cloud' if reducing
  // the edge length by a factor of 1e-2 is not enough.
  for (float high_length = options.max_length();
       high_length > 1e-2f * options.max_length(); high_length /= 2.f) {
    float low_length = high_length / 2.f;
    result = VoxelFilter(point_cloud, low_length);
    if (result.size() >= options.min_num_points()) {
      // Binary search to find the right amount of filtering. 'low_length' gave
      // a sufficiently dense 'result', 'high_length' did not. We stop when the
      // edge length is at most 10% off.
      while ((high_length - low_length) / low_length > 1e-1f) {
        const float mid_length = (low_length + high_length) / 2.f;
        const PointCloud candidate = VoxelFilter(point_cloud, mid_length);
        if (candidate.size() >= options.min_num_points()) {
          low_length = mid_length;
          result = candidate;
        } else {
          high_length = mid_length;
        }
      }
      return result;
    }
  }
  return result;
}

using VoxelKeyType = uint64_t;

// 三轴分别占用21位
VoxelKeyType GetVoxelCellIndex(const Eigen::Vector3f& point,
                               const float resolution) {
  //point对应的每一个参数，都除以　resolution（地图分辨率）
  const Eigen::Array3f index = point.array() / resolution;
  // 对其值四舍五入
  const uint64_t x = common::RoundToInt(index.x());
  const uint64_t y = common::RoundToInt(index.y());
  const uint64_t z = common::RoundToInt(index.z());
  return (x << 42) + (y << 21) + z;
}

template <class T, class PointFunction>
std::vector<bool> RandomizedVoxelFilterIndices(
    const std::vector<T>& point_cloud, const float resolution,
    PointFunction&& point_function) {
  // According to https://en.wikipedia.org/wiki/Reservoir_sampling
  //  随机数生成器
  std::minstd_rand0 generator;
  // VoxelKeyType，根据点坐标和分辨率，计算得到的位置值(x<<42+y<<21+z)
  absl::flat_hash_map<VoxelKeyType, std::pair<int, int>>
      voxel_count_and_point_index;
  
  //遍历所有点 
  for (size_t i = 0; i < point_cloud.size(); i++) {
    // 得到对应位置的　std::pair<int, int>
    auto& voxel = voxel_count_and_point_index[GetVoxelCellIndex(
        point_function(point_cloud[i]), resolution)];
    // 该点数量
    voxel.first++;
    if (voxel.first == 1) {
      // 记录该位置点在点云数据中的索引
      voxel.second = i;
    } else {
      // 随机数分布器，uniform,均匀分布，normal,正态分布；随机数分布范围在　(1,voxel.first)　中．
      std::uniform_int_distribution<> distribution(1, voxel.first);

      // distribution(generator)　生成一个随机数
      if (distribution(generator) == voxel.first) {
        voxel.second = i;
      }
    }
  }

  // 标记所有点云数据,没有被滤掉的数据,置为 true;滤掉数据置为 false
  std::vector<bool> points_used(point_cloud.size(), false);
  for (const auto& voxel_and_index : voxel_count_and_point_index) {
    // 标记出，点云数据中，所有位置点．
    // 筛出多点在同一个格子中的点．即多个点云在同一个格子中有值voxel.first＞１情况，随即选取一个点来标记
    points_used[voxel_and_index.second.second] = true;
  }
  return points_used;
}

template <class T, class PointFunction>
std::vector<T> RandomizedVoxelFilter(const std::vector<T>& point_cloud,
                                     const float resolution,
                                     PointFunction&& point_function) {
  const std::vector<bool> points_used =
      RandomizedVoxelFilterIndices(point_cloud, resolution, point_function);

  std::vector<T> results;
  for (size_t i = 0; i < point_cloud.size(); i++) {
    if (points_used[i]) {
      results.push_back(point_cloud[i]);
    }
  }
  return results;
}

}  // namespace

std::vector<RangefinderPoint> VoxelFilter(
    const std::vector<RangefinderPoint>& points, const float resolution) {
  return RandomizedVoxelFilter(
      points, resolution,
      [](const RangefinderPoint& point) { return point.position; });
}

// 体素滤波
PointCloud VoxelFilter(const PointCloud& point_cloud, const float resolution) {
  // 点云数据筛选标记，所个点云数据在同一个格子中时，随机选取一个点来标记
  // 通过滤波,获取标记状态
  const std::vector<bool> points_used = RandomizedVoxelFilterIndices(
      point_cloud.points(), resolution,
      [](const RangefinderPoint& point) { return point.position; });

  // 根据标记的点，来筛选出可用点云坐标数据
  std::vector<RangefinderPoint> filtered_points;
  for (size_t i = 0; i < point_cloud.size(); i++) {
    if (points_used[i]) {
      // 根据标记状态,选取滤波后的点位置
      filtered_points.push_back(point_cloud[i]);
    }
  }

  // 根据标记的点，来筛选出可用点云索引数据
  std::vector<float> filtered_intensities;
  CHECK_LE(point_cloud.intensities().size(), point_cloud.points().size());
  for (size_t i = 0; i < point_cloud.intensities().size(); i++) {
    if (points_used[i]) {
      // 根据标记状态,选取滤波后的点强度
      filtered_intensities.push_back(point_cloud.intensities()[i]);
    }
  }
  // 返回滤波完成后的点云
  return PointCloud(std::move(filtered_points),
                    std::move(filtered_intensities));
}

TimedPointCloud VoxelFilter(const TimedPointCloud& timed_point_cloud,
                            const float resolution) {
  return RandomizedVoxelFilter(
      timed_point_cloud, resolution,
      [](const TimedRangefinderPoint& point) { return point.position; });
}

std::vector<sensor::TimedPointCloudOriginData::RangeMeasurement> VoxelFilter(
    const std::vector<sensor::TimedPointCloudOriginData::RangeMeasurement>&
        range_measurements,
    const float resolution) {
  return RandomizedVoxelFilter(
      range_measurements, resolution,
      [](const sensor::TimedPointCloudOriginData::RangeMeasurement&
             range_measurement) {
        return range_measurement.point_time.position;
      });
}

proto::AdaptiveVoxelFilterOptions CreateAdaptiveVoxelFilterOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::AdaptiveVoxelFilterOptions options;
  options.set_max_length(parameter_dictionary->GetDouble("max_length"));
  options.set_min_num_points(
      parameter_dictionary->GetNonNegativeInt("min_num_points"));
  options.set_max_range(parameter_dictionary->GetDouble("max_range"));
  return options;
}

PointCloud AdaptiveVoxelFilter(
    const PointCloud& point_cloud,
    const proto::AdaptiveVoxelFilterOptions& options) {

    static bool flag = false;
    if(flag == false)
    {
      LOG(WARNING)
        <<"\n FilterByMaxRange filter:"
        <<"\n options.max_range() = "<<options.max_range();
      flag = true;
    }

  return AdaptivelyVoxelFiltered(
      options, FilterByMaxRange(point_cloud, options.max_range()));
}

}  // namespace sensor
}  // namespace cartographer
