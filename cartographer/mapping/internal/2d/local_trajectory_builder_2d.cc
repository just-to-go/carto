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

#include "cartographer/mapping/internal/2d/local_trajectory_builder_2d.h"

#include <limits>
#include <memory>

#include "absl/memory/memory.h"
#include "cartographer/metrics/family_factory.h"
#include "cartographer/sensor/range_data.h"

namespace cartographer {
namespace mapping {

static auto* kLocalSlamLatencyMetric = metrics::Gauge::Null();
static auto* kLocalSlamRealTimeRatio = metrics::Gauge::Null();
static auto* kLocalSlamCpuRealTimeRatio = metrics::Gauge::Null();
static auto* kRealTimeCorrelativeScanMatcherScoreMetric =
    metrics::Histogram::Null();
static auto* kCeresScanMatcherCostMetric = metrics::Histogram::Null();
static auto* kScanMatcherResidualDistanceMetric = metrics::Histogram::Null();
static auto* kScanMatcherResidualAngleMetric = metrics::Histogram::Null();

LocalTrajectoryBuilder2D::LocalTrajectoryBuilder2D(
    const proto::LocalTrajectoryBuilderOptions2D& options,
    const std::vector<std::string>& expected_range_sensor_ids)
    : options_(options),
      active_submaps_(options.submaps_options()),         //建图
      motion_filter_(options_.motion_filter_options()),   //运动滤波
      real_time_correlative_scan_matcher_(                //实时点云匹配
          options_.real_time_correlative_scan_matcher_options()),
      ceres_scan_matcher_(options_.ceres_scan_matcher_options()),    //非线性优化
      range_data_collator_(expected_range_sensor_ids) {              //多个雷达数据同步

        LOG(WARNING)<<"\n LocalTrajectoryBuilder2D 初始化！";
      }

LocalTrajectoryBuilder2D::~LocalTrajectoryBuilder2D() {}

//transform_to_gravity_aligned_frame = gravity_alignment.cast<float>() * range_data_poses.back().inverse()
sensor::RangeData
LocalTrajectoryBuilder2D::TransformToGravityAlignedFrameAndFilter(
    const transform::Rigid3f& transform_to_gravity_aligned_frame,
    const sensor::RangeData& range_data) const {
  // 对点云数据，进行姿态矫正　和　高度筛选
  // transform_to_gravity_aligned_frame 为变换矩阵 重力对其变换矩阵 Tbaseg_map
  // 重力对齐最后一个点时刻的重力方向
  // Tbaseg_map * Tmap_point = Tbaseg_point  得到相对于重力系的点云坐标
  /*
      struct RangeData {
                        Eigen::Vector3f origin;
                        PointCloud returns;
                        PointCloud misses;
                       };
  */ 
  const sensor::RangeData cropped =
      sensor::CropRangeData(sensor::TransformRangeData(
                                range_data, transform_to_gravity_aligned_frame),
                            options_.min_z(), options_.max_z());
  static bool flag = false;
  if(flag == false)
  {
    flag = true;
    LOG(WARNING)
      <<"\n TransformToGravityAlignedFrameAndFilter param:"
      <<"\n options_.voxel_filter_size() = "<<options_.voxel_filter_size();
  }
  // 进行滤波
  return sensor::RangeData{
      cropped.origin,
      // 体素滤波是点在baseg坐标系下,进行滤波的
      sensor::VoxelFilter(cropped.returns, options_.voxel_filter_size()),
      sensor::VoxelFilter(cropped.misses, options_.voxel_filter_size())};
}

//扫描匹配
/*
    激光扫描匹配
      输入: 
        time 数据帧时刻(点云数据最后一个点对应的时刻)
        pose_prediction 重力对齐后的机器人坐标 Tmap_baseg
        filtered_gravity_aligned_point_cloud 重力对齐后的点云坐标 Tbaseg_point
*/ 
std::unique_ptr<transform::Rigid2d> LocalTrajectoryBuilder2D::ScanMatch(
    const common::Time time, const transform::Rigid2d& pose_prediction,
    const sensor::PointCloud& filtered_gravity_aligned_point_cloud) {
  if (active_submaps_.submaps().empty()) {
    return absl::make_unique<transform::Rigid2d>(pose_prediction);
  }
  // 获取第一个活跃子地图
  std::shared_ptr<const Submap2D> matching_submap =
      active_submaps_.submaps().front();
  // The online correlative scan matcher will refine the initial estimate for
  // the Ceres scan matcher.
  transform::Rigid2d initial_ceres_pose = pose_prediction;

  // LOG(WARNING)<<"\n pose_prediction.cast<float>() = "<<pose_prediction.cast<float>()
  // <<"\n options_.use_online_correlative_scan_matching() = "<<options_.use_online_correlative_scan_matching();

  static bool flag = false;
  if(flag == false)
  {
    LOG(WARNING)
      <<"\n options_.use_online_correlative_scan_matching() = "<<options_.use_online_correlative_scan_matching();
    flag = true;
  }

  if (options_.use_online_correlative_scan_matching()) {
    //跟前一个submap子图进行匹配，获取一个 initial_ceres_pose
    const double score = real_time_correlative_scan_matcher_.Match(
        pose_prediction, filtered_gravity_aligned_point_cloud,
        *matching_submap->grid(), &initial_ceres_pose);
    
    /*
      static auto* kRealTimeCorrelativeScanMatcherScoreMetric = metrics::Histogram::Null();
      ?? 作用 ??
    */
    kRealTimeCorrelativeScanMatcherScoreMetric->Observe(score);
  }

  auto pose_observation = absl::make_unique<transform::Rigid2d>();
  ceres::Solver::Summary summary;

  /*
    做非线性优化，获取pose_observation
      pose_prediction.translation 预测的位置
      initial_ceres_pose 激光实时匹配的最优位姿
      filtered_gravity_aligned_point_cloud 点云数据
      *matching_submap->grid() 栅格地图
  */
  ceres_scan_matcher_.Match(pose_prediction.translation(), initial_ceres_pose,
                            filtered_gravity_aligned_point_cloud,
                            *matching_submap->grid(), pose_observation.get(),
                            &summary);
  if (pose_observation) {
    /*
      ?? 作用 ??
    */ 
    kCeresScanMatcherCostMetric->Observe(summary.final_cost);

    //求 匹配得到的最优解 与 推算出来的预测点 之间的距离误差
    const double residual_distance =
        (pose_observation->translation() - pose_prediction.translation())
            .norm();

    /*
      ?? 作用 ??
    */ 
    kScanMatcherResidualDistanceMetric->Observe(residual_distance);
    
    // 求角度误差
    const double residual_angle =
        std::abs(pose_observation->rotation().angle() -
                 pose_prediction.rotation().angle());

    /*
      ?? 作用 ??
    */ 
    kScanMatcherResidualAngleMetric->Observe(residual_angle);
  }

  //返回最优解
  return pose_observation;
}

// 处理雷达点云数据
std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult>
LocalTrajectoryBuilder2D::AddRangeData(
    const std::string& sensor_id,
    const sensor::TimedPointCloudData& unsynchronized_data) {
  
  //缓存该帧数据，然后截取不同传感器的数据 同一时间段数据，然后按时间排序，统一输出。
  // 这样就将不同雷达数据，统一起来了。针对多个雷达传感器数据
  auto synchronized_data =
      range_data_collator_.AddRangeData(sensor_id, unsynchronized_data);
  if (synchronized_data.ranges.empty()) {
    LOG(INFO) << "Range data collator filling buffer.";
    return nullptr;
  }

  // 获取该帧时间戳数据
  const common::Time& time = synchronized_data.time;
  // Initialize extrapolator now if we do not ever use an IMU.
  if (!options_.use_imu_data()) {
    // 不使用IMU时，初始化推算器
    InitializeExtrapolator(time);
  }

  if (extrapolator_ == nullptr) {
    // Until we've initialized the extrapolator with our first IMU message, we
    // cannot compute the orientation of the rangefinder.
    LOG(INFO) << "Extrapolator not yet initialized.";
    return nullptr;
  }

  CHECK(!synchronized_data.ranges.empty());
  // TODO(gaschler): Check if this can strictly be 0.
  CHECK_LE(synchronized_data.ranges.back().point_time.time, 0.f);

  //计算第一点时间 = 点云获取时间 + 第一个点记录的相对时间
  const common::Time time_first_point =
      time +
      common::FromSeconds(synchronized_data.ranges.front().point_time.time);

  //如果第一个点时间 小于 extrapolator_->AddPose() 队列中最新点时间
  //若没有初始化。则 extrapolator_->GetLastPoseTime() 为最小值
  if (time_first_point < extrapolator_->GetLastPoseTime()) {
    LOG(INFO) << "Extrapolator is still initializing.";
    return nullptr;
  }

  std::vector<transform::Rigid3f> range_data_poses;
  range_data_poses.reserve(synchronized_data.ranges.size());
  bool warned = false;

  //遍历每个点，计算每个点对应时刻的 PoseExtrapolator 推算出来的机器人Pose，放入到 range_data_poses 向量中。
  for (const auto& range : synchronized_data.ranges) {
    common::Time time_point = time + common::FromSeconds(range.point_time.time);  //计算获取对应点的时刻
    if (time_point < extrapolator_->GetLastExtrapolatedTime()) {    //若对应点时刻，小于更新时间，则取目前已经更新的时间
      if (!warned) {
        LOG(ERROR)
            << "Timestamp of individual range data point jumps backwards from "
            << extrapolator_->GetLastExtrapolatedTime() << " to " << time_point;
        warned = true;
      }
      time_point = extrapolator_->GetLastExtrapolatedTime();
    }
    //range_data_poses ，存放推算的pose
    // 通过外插器,推算出 time_point 时刻的机器人位置,然后压入 range_data_poses 中
    range_data_poses.push_back(                                        
        extrapolator_->ExtrapolatePose(time_point).cast<float>());
  }

  if (num_accumulated_ == 0) {
    // 'accumulated_range_data_.origin' is uninitialized until the last
    // accumulation.

    // 地一帧数据，初始化　accumulated_range_data_
    accumulated_range_data_ = sensor::RangeData{{}, {}, {}};
  }

  // Drop any returns below the minimum range and convert returns beyond the
  // maximum range into misses.
  for (size_t i = 0; i < synchronized_data.ranges.size(); ++i) {
    const sensor::TimedRangefinderPoint& hit =       //获取雷达点
        synchronized_data.ranges[i].point_time;

    //计算 该点对应时刻,其雷达到地图坐标系下的坐标
    /*
      range_data_poses[i] = Tmap_base
      synchronized_data.origins.at(synchronized_data.ranges[i].origin_index) = Tbase_laser
      range_data_poses[i]*synchronized_data.origins.at(synchronized_data.ranges[i].origin_index)
      = Tmap_base * Tbase_laser = Tmap_laser
    */ 
    const Eigen::Vector3f origin_in_local =
        //机器人此刻的位姿 
        range_data_poses[i] *
        synchronized_data.origins.at(synchronized_data.ranges[i].origin_index);

    //将hit数据，转换到 local 坐标系下。
    // 计算该时刻点base到地图坐标系下的坐标
    /*
        range_data_poses[i] = Tmap_base
        sensor::ToRangefinderPoint(hit) = Tbase
        hit_in_local = Tmap_base * Tbase 得到点到地图坐标系下的坐标
    */ 
    sensor::RangefinderPoint hit_in_local =
        range_data_poses[i] * sensor::ToRangefinderPoint(hit);

    //local坐标系下，由 origin 到 hit 的向量    
    const Eigen::Vector3f delta = hit_in_local.position - origin_in_local;
    const float range = delta.norm();//求向量的模

    // 向量长度 在 一定范围之内
    if (range >= options_.min_range()) {
      if (range <= options_.max_range()) {

        //距离在合理范围之内，将点压入到 accumulated_range_data_ 的 returns 中
        accumulated_range_data_.returns.push_back(hit_in_local);
      } else {

        static bool log_flag = false;
        if(log_flag == false)
        {
          log_flag = true;
          LOG(WARNING)<<"\n options_.missing_data_ray_length() = "<<options_.missing_data_ray_length();
        }
        hit_in_local.position =
            origin_in_local +
            options_.missing_data_ray_length() / range * delta;

        //距离不在其范围之内，将其压入到 accumulated_range_data_ 的 misses 中
        accumulated_range_data_.misses.push_back(hit_in_local);
      }
    }
  }
  ++num_accumulated_;

  static bool log_flag = false;
  if(log_flag == false)
  {
    log_flag = true;
    LOG(WARNING)<<"\n options_.num_accumulated_range_data() = "<<options_.num_accumulated_range_data();
  }

  if (num_accumulated_ >= options_.num_accumulated_range_data()) {
    // 当接收到的雷达数据帧，大于等于　options_.num_accumulated_range_data()　时，　进行前端建图
    // 开始建图,以最后一帧点云数据,作为建图点云大帧的 帧时间戳
    const common::Time current_sensor_time = synchronized_data.time;
    absl::optional<common::Duration> sensor_duration;
    if (last_sensor_time_.has_value()) {
      sensor_duration = current_sensor_time - last_sensor_time_.value();
    }
    last_sensor_time_ = current_sensor_time;
    num_accumulated_ = 0;

    // 通过外插器计算重力方向
    const transform::Rigid3d gravity_alignment = transform::Rigid3d::Rotation(
        extrapolator_->EstimateGravityOrientation(time));
    // TODO(gaschler): This assumes that 'range_data_poses.back()' is at time
    // 'time'.

    // 以最后一帧数据的最后一个点时刻对应的base在地图坐标系下的坐标,作为建图帧的原点.
    accumulated_range_data_.origin = range_data_poses.back().translation();

    /*
      重力方向:gravity_alignment,重力对齐即将当前所有点坐标,都转换到与该时刻姿态一直的坐标系下.
      设该时刻的变换矩阵为 Tbaseg_base
      range_data_poses,为不同时刻的点的坐标变换 Tmap_base.
      则有:
          gravity_alignment * range_data_poses.back().inverse() = Tbaseg_base * Tbase_map = Tbaseg_map
    */ 
    // LOG(ERROR)
    // <<"\n gravity_alignment.x = "<<gravity_alignment.translation().x()
    // <<"\n gravity_alignment.y = "<<gravity_alignment.translation().y()
    // <<"\n range_data_poses.back().x = "<<range_data_poses.back().translation().x()
    // <<"\n range_data_poses.back().y = "<<range_data_poses.back().translation().y();
    // 局部地图，添加点云数据．局部建图核心函数
    return AddAccumulatedRangeData(
        time,
        TransformToGravityAlignedFrameAndFilter(//进行体素滤波
            gravity_alignment.cast<float>() * range_data_poses.back().inverse(),
            accumulated_range_data_),
        gravity_alignment, sensor_duration);
  }

  // 雷达数据帧　num_accumulated_ < options_.num_accumulated_range_data() 直接返回　nullptr
  return nullptr;
}

// 局部地图，添加点云数据
std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult>
LocalTrajectoryBuilder2D::AddAccumulatedRangeData(
    const common::Time time,
    const sensor::RangeData& gravity_aligned_range_data,  //滤波之后的点云数据,在baseg坐标系下
    const transform::Rigid3d& gravity_alignment,          //计算重力方向
    const absl::optional<common::Duration>& sensor_duration) {
  if (gravity_aligned_range_data.returns.empty()) {
    LOG(WARNING) << "Dropped empty horizontal range data.";
    return nullptr;
  }

  // Computes a gravity aligned pose prediction.
  
  const transform::Rigid3d non_gravity_aligned_pose_prediction =
      extrapolator_->ExtrapolatePose(time);
/*
  template <typename FloatType>
  Rigid3<FloatType> operator*(const Rigid3<FloatType>& lhs,
                              const Rigid3<FloatType>& rhs) {
    return Rigid3<FloatType>(
        lhs.rotation() * rhs.translation() + lhs.translation(),
        (lhs.rotation() * rhs.rotation()).normalized());
  }
*/ 
// 计算重力对齐的　姿态预测
/*
  non_gravity_aligned_pose_prediction 非重力对齐的姿态 Tmap_base
  gravity_alignment 重力方向变换矩阵 Tbaseg_base
  则有:
    non_gravity_aligned_pose_prediction * gravity_alignment.inverse() = Tmap_base * Tbase_baseg
    = Tmap_baseg 得到重力方向下的base坐标,相对地图坐标系下的位姿

  理论上,对其后 pose_prediction 的角度是0,但是由于 non_gravity_aligned_pose_prediction 是通过上一帧pose数据推算出来的姿态
  gravity_alignment 完全是通过IMU数据推算出来的姿态,两者之间会有一定的误差,因此 pose_prediction 的角度不完成等于0
*/ 
  const transform::Rigid2d pose_prediction = transform::Project2D(
      non_gravity_aligned_pose_prediction * gravity_alignment.inverse());

  // LOG(ERROR)
  //   <<"\n gravity_alignment.x = "<<gravity_alignment.translation().x()
  //   <<"\n gravity_alignment.y = "<<gravity_alignment.translation().y()
  //   <<"\n pose_prediction.x = "<<pose_prediction.translation().x()
  //   <<"\n pose_prediction.y = "<<pose_prediction.translation().y()
  //   <<"\n pose_prediction.cast<float>().angle() = "<<pose_prediction.cast<float>();

  // 自适应体素滤波，不断调整，滤波器的分辨率大小，使得滤波之后的点云数量刚好大于最小点云数量
  // gravity_aligned_range_data.returns 已经是重力对齐之后的点云数据了,无需再进行点云对齐操作
  const sensor::PointCloud& filtered_gravity_aligned_point_cloud =
      sensor::AdaptiveVoxelFilter(gravity_aligned_range_data.returns,
                                  options_.adaptive_voxel_filter_options());
  if (filtered_gravity_aligned_point_cloud.empty()) {
    return nullptr;
  }

  // local map frame <- gravity-aligned frame
  // 局部建图，雷达点云数据，与第一个活跃 submap 实时匹配，并进行非线性优化，得到最有坐标
  /*
      pose_prediction : 重力对齐后的预测值
      filtered_gravity_aligned_point_cloud: 重力对齐后的点云数据
  */ 
  std::unique_ptr<transform::Rigid2d> pose_estimate_2d =
      ScanMatch(time, pose_prediction, filtered_gravity_aligned_point_cloud);
  if (pose_estimate_2d == nullptr) {
    LOG(WARNING) << "Scan matching failed.";
    return nullptr;
  }

  /*
      pose_estimate_2d = Tmap_baseg
      gravity_alignment = Tbaseg_base
      pose_estimate = pose_estimate_2d * gravity_alignment 
                    = Tmap_baseg * Tbaseg_base = Tmap_base
  */ 
  const transform::Rigid3d pose_estimate =
      transform::Embed3D(*pose_estimate_2d) * gravity_alignment;
  // 添加节点
  extrapolator_->AddPose(time, pose_estimate); 

  // 点云数据　和　匹配最优坐标，来修正RangeData
  /*
      pose_estimate_2d = Tmap_baseg
      gravity_aligned_range_data = Tbaseg_point
      range_data_in_local = pose_estimate_2d * gravity_aligned_range_data
                          = Tmap_baseg * Tbaseg_point = Tmap_point 得到相对map坐标系的点云数据
  */ 
  sensor::RangeData range_data_in_local =
      TransformRangeData(gravity_aligned_range_data,
                         transform::Embed3D(pose_estimate_2d->cast<float>()));
  
  /*
    向地图中插入雷达帧
      range_data_in_local 修正后相对map坐标系的点云 Tmap_point
      filtered_gravity_aligned_point_cloud 重力对齐的点云数据 Tbaseg_point
  */ 
  std::unique_ptr<InsertionResult> insertion_result = InsertIntoSubmap(
      time, range_data_in_local, filtered_gravity_aligned_point_cloud,
      pose_estimate, gravity_alignment.rotation());

  const auto wall_time = std::chrono::steady_clock::now();
  if (last_wall_time_.has_value()) {
    const auto wall_time_duration = wall_time - last_wall_time_.value();
    kLocalSlamLatencyMetric->Set(common::ToSeconds(wall_time_duration));
    if (sensor_duration.has_value()) {
      kLocalSlamRealTimeRatio->Set(common::ToSeconds(sensor_duration.value()) /
                                   common::ToSeconds(wall_time_duration));
    }
  }
  const double thread_cpu_time_seconds = common::GetThreadCpuTimeSeconds();
  if (last_thread_cpu_time_seconds_.has_value()) {
    const double thread_cpu_duration_seconds =
        thread_cpu_time_seconds - last_thread_cpu_time_seconds_.value();
    if (sensor_duration.has_value()) {
      kLocalSlamCpuRealTimeRatio->Set(
          common::ToSeconds(sensor_duration.value()) /
          thread_cpu_duration_seconds);
    }
  }
  last_wall_time_ = wall_time;
  last_thread_cpu_time_seconds_ = thread_cpu_time_seconds;
  return absl::make_unique<MatchingResult>(
      MatchingResult{time, pose_estimate, std::move(range_data_in_local),
                     std::move(insertion_result)});
}

// 向submap中插入雷达数据帧
std::unique_ptr<LocalTrajectoryBuilder2D::InsertionResult>
LocalTrajectoryBuilder2D::InsertIntoSubmap(
    const common::Time time, const sensor::RangeData& range_data_in_local,
    const sensor::PointCloud& filtered_gravity_aligned_point_cloud,
    const transform::Rigid3d& pose_estimate,
    const Eigen::Quaterniond& gravity_alignment) {
  if (motion_filter_.IsSimilar(time, pose_estimate)) {
    // 时间，空间不满足要求，不更新
    return nullptr;
  }
  // 向子地图插入雷达数据
  std::vector<std::shared_ptr<const Submap2D>> insertion_submaps =
      active_submaps_.InsertRangeData(range_data_in_local);
  
  // 返回插入地图结果
  return absl::make_unique<InsertionResult>(InsertionResult{
      std::make_shared<const TrajectoryNode::Data>(TrajectoryNode::Data{
          time,
          gravity_alignment,
          filtered_gravity_aligned_point_cloud,
          {},  // 'high_resolution_point_cloud' is only used in 3D.
          {},  // 'low_resolution_point_cloud' is only used in 3D.
          {},  // 'rotational_scan_matcher_histogram' is only used in 3D.
          pose_estimate}),
      std::move(insertion_submaps)});
}

// 添加IMU数据,即向外插器中插入 imu数据
void LocalTrajectoryBuilder2D::AddImuData(const sensor::ImuData& imu_data) {
  CHECK(options_.use_imu_data()) << "An unexpected IMU packet was added.";
  InitializeExtrapolator(imu_data.time);
  extrapolator_->AddImuData(imu_data);
}

// 添加里程计数据 , 外插器添加里程计数据
void LocalTrajectoryBuilder2D::AddOdometryData(
    const sensor::OdometryData& odometry_data) {
  if (extrapolator_ == nullptr) {
    // Until we've initialized the extrapolator we cannot add odometry data.
    LOG(INFO) << "Extrapolator not yet initialized.";
    return;
  }
  extrapolator_->AddOdometryData(odometry_data);
}

// 推算器初始化
void LocalTrajectoryBuilder2D::InitializeExtrapolator(const common::Time time) {
  if (extrapolator_ != nullptr) {
    return;
  }
  CHECK(!options_.pose_extrapolator_options().use_imu_based());
  // TODO(gaschler): Consider using InitializeWithImu as 3D does.
  extrapolator_ = absl::make_unique<PoseExtrapolator>(
      ::cartographer::common::FromSeconds(options_.pose_extrapolator_options()
                                              .constant_velocity()
                                              .pose_queue_duration()),
      options_.pose_extrapolator_options()
          .constant_velocity()
          .imu_gravity_time_constant());
  // Identity 全为　０　，初始化　Pose　为　０
  extrapolator_->AddPose(time, transform::Rigid3d::Identity());

  LOG(WARNING)
      <<"\n 推算器初始化　LocalTrajectoryBuilder2D："
      <<"\n options_.pose_extrapolator_options().use_imu_based() = "
      <<options_.pose_extrapolator_options().use_imu_based()
      <<"\n options_.pose_extrapolator_options().constant_velocity().pose_queue_duration() = "
      <<options_.pose_extrapolator_options().constant_velocity().pose_queue_duration()
      <<"\n options_.pose_extrapolator_options().constant_velocity().imu_gravity_time_constant() = "
      <<options_.pose_extrapolator_options().constant_velocity().imu_gravity_time_constant();
}

void LocalTrajectoryBuilder2D::RegisterMetrics(
    metrics::FamilyFactory* family_factory) {
  auto* latency = family_factory->NewGaugeFamily(
      "mapping_2d_local_trajectory_builder_latency",
      "Duration from first incoming point cloud in accumulation to local slam "
      "result");
  kLocalSlamLatencyMetric = latency->Add({});
  auto* real_time_ratio = family_factory->NewGaugeFamily(
      "mapping_2d_local_trajectory_builder_real_time_ratio",
      "sensor duration / wall clock duration.");
  kLocalSlamRealTimeRatio = real_time_ratio->Add({});

  auto* cpu_real_time_ratio = family_factory->NewGaugeFamily(
      "mapping_2d_local_trajectory_builder_cpu_real_time_ratio",
      "sensor duration / cpu duration.");
  kLocalSlamCpuRealTimeRatio = cpu_real_time_ratio->Add({});
  auto score_boundaries = metrics::Histogram::FixedWidth(0.05, 20);
  auto* scores = family_factory->NewHistogramFamily(
      "mapping_2d_local_trajectory_builder_scores", "Local scan matcher scores",
      score_boundaries);
  kRealTimeCorrelativeScanMatcherScoreMetric =
      scores->Add({{"scan_matcher", "real_time_correlative"}});
  auto cost_boundaries = metrics::Histogram::ScaledPowersOf(2, 0.01, 100);
  auto* costs = family_factory->NewHistogramFamily(
      "mapping_2d_local_trajectory_builder_costs", "Local scan matcher costs",
      cost_boundaries);
  kCeresScanMatcherCostMetric = costs->Add({{"scan_matcher", "ceres"}});
  auto distance_boundaries = metrics::Histogram::ScaledPowersOf(2, 0.01, 10);
  auto* residuals = family_factory->NewHistogramFamily(
      "mapping_2d_local_trajectory_builder_residuals",
      "Local scan matcher residuals", distance_boundaries);
  kScanMatcherResidualDistanceMetric =
      residuals->Add({{"component", "distance"}});
  kScanMatcherResidualAngleMetric = residuals->Add({{"component", "angle"}});
}

}  // namespace mapping
}  // namespace cartographer
