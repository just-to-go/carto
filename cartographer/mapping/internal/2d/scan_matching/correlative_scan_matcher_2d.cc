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

#include "cartographer/mapping/internal/2d/scan_matching/correlative_scan_matcher_2d.h"

#include <cmath>

#include "cartographer/common/math.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

// 构建搜索参数,构建搜索组
SearchParameters::SearchParameters(const double linear_search_window,
                                   const double angular_search_window,
                                   const sensor::PointCloud& point_cloud,
                                   const double resolution)
    : resolution(resolution) {
  // We set this value to something on the order of resolution to make sure that
  // the std::acos() below is defined.
  // 雷达数据最小长度距离，为３倍的像素距离
  float max_scan_range = 3.f * resolution;
  // 计算最大的激光点束长度,用于计算角度步长
  for (const sensor::RangefinderPoint& point : point_cloud) {
    // 点长度
    const float range = point.position.head<2>().norm();
    max_scan_range = std::max(range, max_scan_range);
  }
  const double kSafetyMargin = 1. - 1e-3;
  // 计算角度扰动步长
  /*
    利用三角形余弦定理,计算角度步长.其中角度两条长边,做了近似相等的处理 a=b
    cos = (a^2 + b^2 - c^2)/2ab = (2a^2-c^2)/2a^2 = (1-c^2/2a^2)
  */ 
  angular_perturbation_step_size =
      kSafetyMargin * std::acos(1. - common::Pow2(resolution) /
                                         (2. * common::Pow2(max_scan_range)));
  // 计算角度扰动数量
  num_angular_perturbations =
      std::ceil(angular_search_window / angular_perturbation_step_size);
  num_scans = 2 * num_angular_perturbations + 1;

  // 计算线性扰动数量
  const int num_linear_perturbations =
      std::ceil(linear_search_window / resolution);

  static bool flag = false;
  if(flag == false)
  {
    LOG(WARNING)
      <<"\n angular_perturbation_step_size = "<<angular_perturbation_step_size
      <<"\n num_angular_perturbations = "<<num_angular_perturbations
      <<"\n num_scans = "<<num_scans
      <<"\n num_linear_perturbations = "<<num_linear_perturbations;

    flag = true;
  }

  // 分 num_scans 组
  linear_bounds.reserve(num_scans);
  for (int i = 0; i != num_scans; ++i) {
    linear_bounds.push_back(
        LinearBounds{-num_linear_perturbations, num_linear_perturbations,
                     -num_linear_perturbations, num_linear_perturbations});
  }
}

SearchParameters::SearchParameters(const int num_linear_perturbations,
                                   const int num_angular_perturbations,
                                   const double angular_perturbation_step_size,
                                   const double resolution)
    : num_angular_perturbations(num_angular_perturbations),
      angular_perturbation_step_size(angular_perturbation_step_size),
      resolution(resolution),
      num_scans(2 * num_angular_perturbations + 1) {
  linear_bounds.reserve(num_scans);
  for (int i = 0; i != num_scans; ++i) {
    linear_bounds.push_back(
        LinearBounds{-num_linear_perturbations, num_linear_perturbations,
                     -num_linear_perturbations, num_linear_perturbations});
  }
}

void SearchParameters::ShrinkToFit(const std::vector<DiscreteScan2D>& scans,
                                   const CellLimits& cell_limits) {
  CHECK_EQ(scans.size(), num_scans);
  CHECK_EQ(linear_bounds.size(), num_scans);
  for (int i = 0; i != num_scans; ++i) {
    Eigen::Array2i min_bound = Eigen::Array2i::Zero();
    Eigen::Array2i max_bound = Eigen::Array2i::Zero();
    for (const Eigen::Array2i& xy_index : scans[i]) {
      min_bound = min_bound.min(-xy_index);
      max_bound = max_bound.max(Eigen::Array2i(cell_limits.num_x_cells - 1,
                                               cell_limits.num_y_cells - 1) -
                                xy_index);
    }
    linear_bounds[i].min_x = std::max(linear_bounds[i].min_x, min_bound.x());
    linear_bounds[i].max_x = std::min(linear_bounds[i].max_x, max_bound.x());
    linear_bounds[i].min_y = std::max(linear_bounds[i].min_y, min_bound.y());
    linear_bounds[i].max_y = std::min(linear_bounds[i].max_y, max_bound.y());
  }
}

// 构建所有旋转雷达帧
std::vector<sensor::PointCloud> GenerateRotatedScans(
    const sensor::PointCloud& point_cloud,
    const SearchParameters& search_parameters) {
  std::vector<sensor::PointCloud> rotated_scans;
  // 根据角度搜索组，来构建不同的旋转雷达帧
  rotated_scans.reserve(search_parameters.num_scans);

  double delta_theta = -search_parameters.num_angular_perturbations *
                       search_parameters.angular_perturbation_step_size;
  for (int scan_index = 0; scan_index < search_parameters.num_scans;
       ++scan_index,
           delta_theta += search_parameters.angular_perturbation_step_size) {
    //  雷达帧点云数据，与该点搜索角度对齐
    rotated_scans.push_back(sensor::TransformPointCloud(
        point_cloud, transform::Rigid3f::Rotation(Eigen::AngleAxisf(
                         delta_theta, Eigen::Vector3f::UnitZ()))));
  }
  return rotated_scans;
}

// 计算不同组点云帧在预测点位置时在栅格地图中的索引
std::vector<DiscreteScan2D> DiscretizeScans(
    const MapLimits& map_limits, const std::vector<sensor::PointCloud>& scans,
    const Eigen::Translation2f& initial_translation) {
  std::vector<DiscreteScan2D> discrete_scans;
  discrete_scans.reserve(scans.size());
  for (const sensor::PointCloud& scan : scans) {
    discrete_scans.emplace_back();
    discrete_scans.back().reserve(scan.size());
    for (const sensor::RangefinderPoint& point : scan) {
      // 点云数据，平移到 initial_translation
      /*
        initial_translation 预测点位置坐标 tmap_baseg
        point.position.head<2>() 点坐标 tbaseg_point
        translated_point = initial_translation * point.position.head<2>() = tmap_baseg * tbaseg_point = tmap_point
      */ 
      const Eigen::Vector2f translated_point =
          Eigen::Affine2f(initial_translation) * point.position.head<2>();
      // 将点云在地图中位置索引，压入　discrete_scans

      /*
        计算 点坐标在栅格地图中的索引,压入到 discrete_scans 中.
      */ 
      discrete_scans.back().push_back(
          map_limits.GetCellIndex(translated_point));
    }
  }
  return discrete_scans;
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
