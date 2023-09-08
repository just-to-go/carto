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

#include "cartographer/mapping/internal/2d/scan_matching/real_time_correlative_scan_matcher_2d.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

#include "Eigen/Geometry"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/math.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/internal/2d/tsdf_2d.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace {

float ComputeCandidateScore(const TSDF2D& tsdf,
                            const DiscreteScan2D& discrete_scan,
                            int x_index_offset, int y_index_offset) {
  float candidate_score = 0.f;
  float summed_weight = 0.f;
  for (const Eigen::Array2i& xy_index : discrete_scan) {
    const Eigen::Array2i proposed_xy_index(xy_index.x() + x_index_offset,
                                           xy_index.y() + y_index_offset);
    const std::pair<float, float> tsd_and_weight =
        tsdf.GetTSDAndWeight(proposed_xy_index);
    const float normalized_tsd_score =
        (tsdf.GetMaxCorrespondenceCost() - std::abs(tsd_and_weight.first)) /
        tsdf.GetMaxCorrespondenceCost();
    const float weight = tsd_and_weight.second;
    candidate_score += normalized_tsd_score * weight;
    summed_weight += weight;
  }
  if (summed_weight == 0.f) return 0.f;
  candidate_score /= summed_weight;
  CHECK_GE(candidate_score, 0.f);
  return candidate_score;
}

// 打分，得到点云数据在地图中的平均概率
float ComputeCandidateScore(const ProbabilityGrid& probability_grid,
                            const DiscreteScan2D& discrete_scan,
                            int x_index_offset, int y_index_offset) {
  float candidate_score = 0.f;
  // 遍历所有的点云数据，拿到其每点在地图中的索引
  for (const Eigen::Array2i& xy_index : discrete_scan) {
    // 获取偏移后的点
    const Eigen::Array2i proposed_xy_index(xy_index.x() + x_index_offset,
                                           xy_index.y() + y_index_offset);
    // 获取点云坐标对应的概率(占据概率)
    const float probability =
        probability_grid.GetProbability(proposed_xy_index);
    // 计算所有点概率和
    candidate_score += probability;
  }
  // 计算平均概率
  candidate_score /= static_cast<float>(discrete_scan.size());
  CHECK_GT(candidate_score, 0.f);
  return candidate_score;
}

}  // namespace

RealTimeCorrelativeScanMatcher2D::RealTimeCorrelativeScanMatcher2D(
    const proto::RealTimeCorrelativeScanMatcherOptions& options)
    : options_(options) {}

// 计算候选帧,将角度,x偏差,y偏差以及搜索参数 存在候选容器中
std::vector<Candidate2D>
RealTimeCorrelativeScanMatcher2D::GenerateExhaustiveSearchCandidates(
    const SearchParameters& search_parameters) const {
  int num_candidates = 0;
  // 计算搜索帧数，每一个角度对应　num_candidates　多帧．一共　num_candidates　＊　num_scans　帧数据
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    const int num_linear_x_candidates =
        (search_parameters.linear_bounds[scan_index].max_x -
         search_parameters.linear_bounds[scan_index].min_x + 1);
    const int num_linear_y_candidates =
        (search_parameters.linear_bounds[scan_index].max_y -
         search_parameters.linear_bounds[scan_index].min_y + 1);
    num_candidates += num_linear_x_candidates * num_linear_y_candidates;
  }
  std::vector<Candidate2D> candidates;
  candidates.reserve(num_candidates);
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    for (int x_index_offset = search_parameters.linear_bounds[scan_index].min_x;
         x_index_offset <= search_parameters.linear_bounds[scan_index].max_x;
         ++x_index_offset) {
      for (int y_index_offset =
               search_parameters.linear_bounds[scan_index].min_y;
           y_index_offset <= search_parameters.linear_bounds[scan_index].max_y;
           ++y_index_offset) {

        //  计算候选帧，scan_index，对应一个角度．x_index_offset,y_index_offset对应位置
        candidates.emplace_back(scan_index, x_index_offset, y_index_offset,
                                search_parameters);
      }
    }
  }
  CHECK_EQ(candidates.size(), num_candidates);
  return candidates;
}

// 前端，实时相关扫描匹配器
/*
  initial_pose_estimate 重力对齐后的机器人坐标 Tmap_baseg.由于存在一定的误差,导致对其后的角度并不完成等于0
  因此,需要
  point_cloud 重力对齐后的点云坐标 Tbaseg_point
*/ 
double RealTimeCorrelativeScanMatcher2D::Match(
    const transform::Rigid2d& initial_pose_estimate,
    const sensor::PointCloud& point_cloud, const Grid2D& grid,
    transform::Rigid2d* pose_estimate) const {
  CHECK(pose_estimate != nullptr);
  // 获取 se2 中的　rotation
  // initial_rotation = Rmap_baseg
  const Eigen::Rotation2Dd initial_rotation = initial_pose_estimate.rotation();

  // 旋转点云坐标,将其与预测姿态对齐
  /*
    预测姿态相对于绝对map的角度
      Rmap_baseg = Eigen::AngleAxisf(initial_rotation.cast<float>().angle(), Eigen::Vector3f::UnitZ())
    point_cloud = Tbaseg_point
    TransformPointCloud = Rmap'_baseg * Tbaseg_point = Tmap'_point
  */ 
  const sensor::PointCloud rotated_point_cloud = sensor::TransformPointCloud(
      point_cloud,
      transform::Rigid3f::Rotation(Eigen::AngleAxisf(
          initial_rotation.cast<float>().angle(), Eigen::Vector3f::UnitZ())));

  // LOG(ERROR)
  // <<"\n Match debug"
  // <<"\n initial_pose_estimate.cast<float>() = "<<initial_pose_estimate.cast<float>()
  // <<"\n initial_rotation.cast<float>().angle() = "<<initial_rotation.cast<float>().angle();

  static bool flag = false;
  if(flag == false)
  {
    flag =true;
    LOG(WARNING)
      <<"\n search_parameters:"
      <<"\n options_.linear_search_window() = "<<options_.linear_search_window()
      <<"\n options_.angular_search_window() = "<<options_.angular_search_window()
      <<"\n grid.limits().resolution() = "<<grid.limits().resolution();
  }
  // 设置搜索参数，构建搜索组，确定角度所有范围　和　位置搜索范围
  // 通过设置搜索窗口,地图分辨率,来计算 角度搜索步长 和 距离搜索步长
  /*
      linear_search_window 线性搜索窗口
      angular_search_window 角度搜索窗口
      rotated_point_cloud 旋转对齐后的点云数据
      resolution 地图分辨率
  */ 
  const SearchParameters search_parameters(
      options_.linear_search_window(), options_.angular_search_window(),
      rotated_point_cloud, grid.limits().resolution());
  
  // 构建不同角度的旋转雷达帧的点云祖
  const std::vector<sensor::PointCloud> rotated_scans =
      GenerateRotatedScans(rotated_point_cloud, search_parameters);

  // 不同搜索雷达帧，平移变换．然后转换成　地图索引　保存在　discrete_scans　中
  // 将不同角度的雷达帧,都平移到预测点位置,将其坐标换算成栅格地图中的索引
  const std::vector<DiscreteScan2D> discrete_scans = DiscretizeScans(
      grid.limits(), rotated_scans,
      Eigen::Translation2f(initial_pose_estimate.translation().x(),
                           initial_pose_estimate.translation().y()));
  
  /*
    生成详尽的搜索候选帧
      Candidate2D:保存每一帧的角度,x偏移量,y偏移量.
  */ 
  std::vector<Candidate2D> candidates =
      GenerateExhaustiveSearchCandidates(search_parameters);

  // 对所有候选帧进行打分
  ScoreCandidates(grid, discrete_scans, search_parameters, &candidates);

  // 排序，并取最高分值候选帧，为最佳候选帧
  const Candidate2D& best_candidate =
      *std::max_element(candidates.begin(), candidates.end());
  
  // 计算最有匹配 Pose
  *pose_estimate = transform::Rigid2d(
      {initial_pose_estimate.translation().x() + best_candidate.x,
       initial_pose_estimate.translation().y() + best_candidate.y},
      initial_rotation * Eigen::Rotation2Dd(best_candidate.orientation));
  
  // 返回最佳候选帧评分
  return best_candidate.score;
}

/*
  grid 栅格地图
  discrete_scans 旋转点云,在栅格中的索引
  search_parameters 搜索参数
  candidates 候选搜索帧
*/ 
void RealTimeCorrelativeScanMatcher2D::ScoreCandidates(
    const Grid2D& grid, const std::vector<DiscreteScan2D>& discrete_scans,
    const SearchParameters& search_parameters,
    std::vector<Candidate2D>* const candidates) const {
  for (Candidate2D& candidate : *candidates) {

    static bool flag = false;
    if(flag == false)
    {
      flag = true;
      LOG(WARNING)
        <<"\n ScoreCandidates:"
        // <<"\n grid.GetGridType() = "<<grid.GetGridType()
        // <<"\n GridType::PROBABILITY_GRID = "<<GridType::PROBABILITY_GRID
        // <<"\n GridType::TSDF = "<<GridType::TSDF
        <<"\n options_.translation_delta_cost_weight() = "<<options_.translation_delta_cost_weight()
        <<"\n options_.rotation_delta_cost_weight() = "<<options_.rotation_delta_cost_weight();
    }
    // 根据不同的地图类型,进行不同的匹配
    switch (grid.GetGridType()) {
      case GridType::PROBABILITY_GRID:
      {
        // 概率地图
        static bool log_flag = false;
        if(log_flag == false)
        {
          LOG(WARNING)<<"\n PROBABILITY_GRID ComputeCandidateScore!";
          log_flag = true;
        }
        /*
          计算该候选帧的评分
            grid 栅格地图
            discrete_scans[candidate.scan_index] 旋转点云
            candidate.x_index_offset x轴偏移量
            candidate.y_index_offset y轴偏移量

            candidate.score 该帧点分数的平均值
        */ 
        candidate.score = ComputeCandidateScore(
            static_cast<const ProbabilityGrid&>(grid),
            discrete_scans[candidate.scan_index], candidate.x_index_offset,
            candidate.y_index_offset);
      }
      break;
      case GridType::TSDF:
      {
        // 三维建图使用的地图
        static bool log_flag = false;
        if(log_flag == false)
        {
          LOG(WARNING)<<"\n TSDF ComputeCandidateScore!";
          log_flag = true;
        }
        candidate.score = ComputeCandidateScore(
            static_cast<const TSDF2D&>(grid),
            discrete_scans[candidate.scan_index], candidate.x_index_offset,
            candidate.y_index_offset);
      }
      break;
    }
    /*
      计算候选帧分值
        candidate.x  x轴偏移量float ;candidate.x_index_offset x轴偏移的栅格
        candidate.y  y轴偏移量float ;candidate.y_index_offset y轴偏移的栅格
        std::exp(a)   e的a次幂函数.a=偏移距离*权重+偏移角度*权重
      候选帧最后分值 = 地图匹配分值 * 权重;偏移量越小权重越大,偏移量为0时,权重为1
    */ 
    candidate.score *=
        std::exp(-common::Pow2(std::hypot(candidate.x, candidate.y) *
                                   options_.translation_delta_cost_weight() +
                               std::abs(candidate.orientation) *
                                   options_.rotation_delta_cost_weight()));
  }
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
