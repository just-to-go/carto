/*
 * Copyright 2017 The Cartographer Authors
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

#include "cartographer/mapping/pose_extrapolator.h"

#include <algorithm>

#include "absl/memory/memory.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
/*
  外推器初始化
    pose_queue_duration 位置队列持续时间
    imu_gravity_time_constant imu重力时间常量
    cached_extrapolated_pose_ 缓存估计的pose
*/ 
PoseExtrapolator::PoseExtrapolator(const common::Duration pose_queue_duration,
                                   double imu_gravity_time_constant)
    : pose_queue_duration_(pose_queue_duration),
      gravity_time_constant_(imu_gravity_time_constant),
      cached_extrapolated_pose_{common::Time::min(),
                                transform::Rigid3d::Identity()} {}

// 3D建图初始化时,使用IMU初始化外推器
std::unique_ptr<PoseExtrapolator> PoseExtrapolator::InitializeWithImu(
    const common::Duration pose_queue_duration,
    const double imu_gravity_time_constant, const sensor::ImuData& imu_data) {
  auto extrapolator = absl::make_unique<PoseExtrapolator>(
      pose_queue_duration, imu_gravity_time_constant);
  extrapolator->AddImuData(imu_data);
  extrapolator->imu_tracker_ =
      absl::make_unique<ImuTracker>(imu_gravity_time_constant, imu_data.time);
  extrapolator->imu_tracker_->AddImuLinearAccelerationObservation(
      imu_data.linear_acceleration);
  extrapolator->imu_tracker_->AddImuAngularVelocityObservation(
      imu_data.angular_velocity);
  extrapolator->imu_tracker_->Advance(imu_data.time);
  extrapolator->AddPose(
      imu_data.time,
      transform::Rigid3d::Rotation(extrapolator->imu_tracker_->orientation()));
  return extrapolator;
}

// 获取最新添加进来的激光Pose时间
common::Time PoseExtrapolator::GetLastPoseTime() const {
  if (timed_pose_queue_.empty()) {
    return common::Time::min();
  }
  return timed_pose_queue_.back().time;
}

// 获取上次推算时间
common::Time PoseExtrapolator::GetLastExtrapolatedTime() const {
  if (!extrapolation_imu_tracker_) {
    return common::Time::min();
  }
  return extrapolation_imu_tracker_->time();
}

/*
  添加激光匹配的Pose
    更新角速度和线速度
    更新 IMU 跟踪器
    裁剪 IMU,Odom 数据
    更新 odometry_imu_tracker_ 和 extrapolation_imu_tracker_
*/ 
void PoseExtrapolator::AddPose(const common::Time time,
                               const transform::Rigid3d& pose) {
  if (imu_tracker_ == nullptr) {
    common::Time tracker_start = time;
    if (!imu_data_.empty()) {
      tracker_start = std::min(tracker_start, imu_data_.front().time);
    }
    imu_tracker_ =
        absl::make_unique<ImuTracker>(gravity_time_constant_, tracker_start);
  }
  timed_pose_queue_.push_back(TimedPose{time, pose});
  while (timed_pose_queue_.size() > 2 &&
         timed_pose_queue_[1].time <= time - pose_queue_duration_) {
    timed_pose_queue_.pop_front();
  }
  UpdateVelocitiesFromPoses();
  // 接收真值，更新　imu_tracker_
  AdvanceImuTracker(time, imu_tracker_.get());
  TrimImuData();
  TrimOdometryData();

  /**
   * 更新完imu_tracker_，同步odometry_imu_tracker_，extrapolation_imu_tracker_
   * **/ 
  odometry_imu_tracker_ = absl::make_unique<ImuTracker>(*imu_tracker_);
  extrapolation_imu_tracker_ = absl::make_unique<ImuTracker>(*imu_tracker_);
}

// 外插器,添加 IMU 数据(时间,三轴角速度,三轴加速度)
void PoseExtrapolator::AddImuData(const sensor::ImuData& imu_data) {
  CHECK(timed_pose_queue_.empty() ||
        imu_data.time >= timed_pose_queue_.back().time);
  imu_data_.push_back(imu_data);

  // 修剪IMU数据
  TrimImuData();
}

// 推算器添加里程计数据，计算线速度和角速度
void PoseExtrapolator::AddOdometryData(
    const sensor::OdometryData& odometry_data) {
  CHECK(timed_pose_queue_.empty() ||
        odometry_data.time >= timed_pose_queue_.back().time);
  // 将数据压入　odometry_data_ 中
  odometry_data_.push_back(odometry_data);

  // 里程计数据修剪，删除时间小于　timed_pose_queue_最新数据时间的点
  TrimOdometryData();
  if (odometry_data_.size() < 2) {
    return;
  }
  // TODO(whess): Improve by using more than just the last two odometry poses.
  // Compute extrapolation in the tracking frame.
  /*
  struct OdometryData {
                        common::Time time;
                        transform::Rigid3d pose;
                      };
  */ 
  const sensor::OdometryData& odometry_data_oldest = odometry_data_.front();
  const sensor::OdometryData& odometry_data_newest = odometry_data_.back();

  // 计算时间,使用前一帧的时间 减去 后一帧的时间,故时间是赋值
  const double odometry_time_delta =
      common::ToSeconds(odometry_data_oldest.time - odometry_data_newest.time);

  // LOG(ERROR)<<"\n odometry_time_delta = "<<odometry_time_delta;
  /*
    odometry_data_newest.pose = Tmap_new
    odometry_data_oldest.pose = Tmap_old
    odometry_data_newest.pose.inverse() * odometry_data_oldest.pose = Tnew_map * Tmap_old = Tnew_old
  */ 
  const transform::Rigid3d odometry_pose_delta =
      odometry_data_newest.pose.inverse() * odometry_data_oldest.pose;

  /*
    四元数转换成旋转变量
  */
  angular_velocity_from_odometry_ =
      transform::RotationQuaternionToAngleAxisVector(
          odometry_pose_delta.rotation()) /
      odometry_time_delta;
  if (timed_pose_queue_.empty()) {
    return;
  }

  // 计算线速度，两帧里程计之间的平均线速度
  const Eigen::Vector3d
      linear_velocity_in_tracking_frame_at_newest_odometry_time =
          odometry_pose_delta.translation() / odometry_time_delta;      //里程计数据计算平均速度
  
  // 计算odometry_data_newest.time时刻，姿态四元数
  /**
      timed_pose_queue_.back().pose.rotation() = Rmap_pose
      ExtrapolateRotation 计算从pose时刻到time时刻,旋转变化量

      orientation_at_newest_odometry_time 表示 time时刻的姿态(激光位姿修正后的姿态)
   * **/ 
  const Eigen::Quaterniond orientation_at_newest_odometry_time =
      timed_pose_queue_.back().pose.rotation() *
      ExtrapolateRotation(odometry_data_newest.time,
                          odometry_imu_tracker_.get());
  /*
    获取最新时刻的速度:相对与最新时刻,机器人本体的速度
      通过 激光匹配位姿 修正后的速度
  */ 
  linear_velocity_from_odometry_ =
      orientation_at_newest_odometry_time *
      linear_velocity_in_tracking_frame_at_newest_odometry_time;

  static double max_v = -1;
  double vel_x = linear_velocity_in_tracking_frame_at_newest_odometry_time(0);
  double vel_y = linear_velocity_in_tracking_frame_at_newest_odometry_time(1);
  double v = hypot(vel_x,vel_y);
  if(v > max_v)
  {
    max_v = v;
    // LOG(ERROR)
    //   <<"\n vel_x = "<<vel_x
    //   <<"\n vel_y = "<<vel_y
    //   <<"\n max_v = "<<max_v
    //   <<"\n odometry_time_delta = "<<odometry_time_delta
    //   <<"\n odometry_data_oldest.x = "<<odometry_data_oldest.pose.translation().x()
    //   <<"\n odometry_data_oldest.y = "<<odometry_data_oldest.pose.translation().y()
    //   <<"\n odometry_data_newest.x = "<<odometry_data_newest.pose.translation().x()
    //   <<"\n odometry_data_newest.y = "<<odometry_data_newest.pose.translation().y()
    //   <<"\n odometry_pose_delta.x = "<<odometry_pose_delta.translation().x()
    //   <<"\n odometry_pose_delta.y = "<<odometry_pose_delta.translation().y();
  }
}

// 推算对应时刻位姿,以上一次 Pose 为基准,计算相对的 trans 和 rotation 
transform::Rigid3d PoseExtrapolator::ExtrapolatePose(const common::Time time) {
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();
  CHECK_GE(time, newest_timed_pose.time);
  if (cached_extrapolated_pose_.time != time) {
    const Eigen::Vector3d translation =
        ExtrapolateTranslation(time) + newest_timed_pose.pose.translation();

    /*
      newest_timed_pose.pose.rotation() = Rmap_pose
        该旋转四元数,是激光匹配得到的姿态.与imu_tracker得到的该时刻姿态会不一致
      ExtrapolateRotation(time, extrapolation_imu_tracker_.get())
        得到的是从 pose时刻到time时刻 的旋转四元数.即取时间段中间的 相对旋转四元数
      rotation 上面两者相乘,表示在激光匹配结果基础上,再旋转一角度,得到我们需要的time时刻的姿态

    */ 
    const Eigen::Quaterniond rotation =
        newest_timed_pose.pose.rotation() *
        ExtrapolateRotation(time, extrapolation_imu_tracker_.get());
    cached_extrapolated_pose_ =
        TimedPose{time, transform::Rigid3d{translation, rotation}};
  }
  return cached_extrapolated_pose_.pose;
}

// 估计 time 时刻姿态,返回对应的姿态四元数
Eigen::Quaterniond PoseExtrapolator::EstimateGravityOrientation(
    const common::Time time) {
  ImuTracker imu_tracker = *imu_tracker_;
  AdvanceImuTracker(time, &imu_tracker);
  return imu_tracker.orientation();
}

// 通过激光Pose,更新角速度和线速度
void PoseExtrapolator::UpdateVelocitiesFromPoses() {
  if (timed_pose_queue_.size() < 2) {
    // We need two poses to estimate velocities.
    return;
  }
  CHECK(!timed_pose_queue_.empty());
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();
  const auto newest_time = newest_timed_pose.time;
  const TimedPose& oldest_timed_pose = timed_pose_queue_.front();
  const auto oldest_time = oldest_timed_pose.time;
  const double queue_delta = common::ToSeconds(newest_time - oldest_time);
  if (queue_delta < common::ToSeconds(pose_queue_duration_)) {
    LOG(WARNING) << "Queue too short for velocity estimation. Queue duration: "
                 << queue_delta << " s";
    return;
  }
  const transform::Rigid3d& newest_pose = newest_timed_pose.pose;
  const transform::Rigid3d& oldest_pose = oldest_timed_pose.pose;
  linear_velocity_from_poses_ =
      (newest_pose.translation() - oldest_pose.translation()) / queue_delta;
  angular_velocity_from_poses_ =
      transform::RotationQuaternionToAngleAxisVector(
          oldest_pose.rotation().inverse() * newest_pose.rotation()) /
      queue_delta;
}

/*
  裁剪IMU数据,删除时间小于　timed_pose_queue_最新数据时间的点
*/ 
void PoseExtrapolator::TrimImuData() {
  while (imu_data_.size() > 1 && !timed_pose_queue_.empty() &&
         imu_data_[1].time <= timed_pose_queue_.back().time) {
    imu_data_.pop_front();
  }
}

/*
  里程计数据修剪，删除时间小于　timed_pose_queue_最新数据时间的点
*/ 
void PoseExtrapolator::TrimOdometryData() {
  while (odometry_data_.size() > 2 && !timed_pose_queue_.empty() &&
         odometry_data_[1].time <= timed_pose_queue_.back().time) {
    odometry_data_.pop_front();
  }
}

/*
  将　imu_tracker　更新到 time 时刻．
    case1: time 之前没有 IMU 数据.使用 odom 或 pose 角速度来更新
    case2: time 之前有IMU数据,上一次更新时间小于IMU最早数据;
    case3: time 之前有IMU数据,上一次更新时间也在IMU数据范围内
*/ 
void PoseExtrapolator::AdvanceImuTracker(const common::Time time,
                                         ImuTracker* const imu_tracker) const {
  CHECK_GE(time, imu_tracker->time());
  if (imu_data_.empty() || time < imu_data_.front().time) {
    // There is no IMU data until 'time', so we advance the ImuTracker and use
    // the angular velocities from poses and fake gravity to help 2D stability.
    imu_tracker->Advance(time);
    imu_tracker->AddImuLinearAccelerationObservation(Eigen::Vector3d::UnitZ());
    imu_tracker->AddImuAngularVelocityObservation(
        odometry_data_.size() < 2 ? angular_velocity_from_poses_
                                  : angular_velocity_from_odometry_);
    return;
  }
  if (imu_tracker->time() < imu_data_.front().time) {
    // Advance to the beginning of 'imu_data_'.
    imu_tracker->Advance(imu_data_.front().time);
  }
  // 找到上一次更新时刻对应的IMU数据
  auto it = std::lower_bound(
      imu_data_.begin(), imu_data_.end(), imu_tracker->time(),
      [](const sensor::ImuData& imu_data, const common::Time& time) {
        return imu_data.time < time;
      });
  while (it != imu_data_.end() && it->time < time) {
    imu_tracker->Advance(it->time);
    imu_tracker->AddImuLinearAccelerationObservation(it->linear_acceleration);
    imu_tracker->AddImuAngularVelocityObservation(it->angular_velocity);
    ++it;
  }
  imu_tracker->Advance(time);
}

// 推算姿态
Eigen::Quaterniond PoseExtrapolator::ExtrapolateRotation(
    const common::Time time, ImuTracker* const imu_tracker) const {
  CHECK_GE(time, imu_tracker->time());

  // 更新 imu_tracker 姿态到　time 时刻．
  AdvanceImuTracker(time, imu_tracker);

  // 上一个Pose时刻的　姿态
  const Eigen::Quaterniond last_orientation = imu_tracker_->orientation();

  /*
    imu_tracker->orientation() time时刻姿态.陀螺机计算出来的位姿 Rmap_time
    last_orientation.inverse() 为pose时刻,陀螺仪计算的姿态,Rmap_pose1
    Rmap_pose1 * Rpose1_time = Rmap_time ==>Rpose1_time = Rmap_pose1.inverse() * Rmap_time
    即返回,从pose时刻,到time时刻之间的旋转四元数
  */ 
  return last_orientation.inverse() * imu_tracker->orientation();
}

Eigen::Vector3d PoseExtrapolator::ExtrapolateTranslation(common::Time time) {
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();
  const double extrapolation_delta =
      common::ToSeconds(time - newest_timed_pose.time);

  static double max_time = -1;
  if(extrapolation_delta > max_time)
  {
    max_time = extrapolation_delta;
    LOG(WARNING)
      <<"\n extrapolation_delta = "<<extrapolation_delta
      <<"\n time = "<<time
      <<"\n newest_timed_pose.time = "<<newest_timed_pose.time;
  }

  if (odometry_data_.size() < 2)
  {
    return extrapolation_delta * linear_velocity_from_poses_;
  }
  return extrapolation_delta * linear_velocity_from_odometry_;
}

// 用于3D 建图时调用
PoseExtrapolator::ExtrapolationResult
PoseExtrapolator::ExtrapolatePosesWithGravity(
    const std::vector<common::Time>& times) {
  std::vector<transform::Rigid3f> poses;
  for (auto it = times.begin(); it != std::prev(times.end()); ++it) {
    poses.push_back(ExtrapolatePose(*it).cast<float>());
  }

  const Eigen::Vector3d current_velocity = odometry_data_.size() < 2
                                               ? linear_velocity_from_poses_
                                               : linear_velocity_from_odometry_;
  return ExtrapolationResult{poses, ExtrapolatePose(times.back()),
                             current_velocity,
                             EstimateGravityOrientation(times.back())};
}

}  // namespace mapping
}  // namespace cartographer
