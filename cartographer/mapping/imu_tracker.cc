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

#include "cartographer/mapping/imu_tracker.h"

#include <cmath>
#include <limits>

#include "cartographer/common/math.h"
#include "cartographer/mapping/internal/eigen_quaterniond_from_two_vectors.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

/*
  IMU 跟踪器
    imu_gravity_time_constant_ 重力时间常数
    time_ 更新姿态和重力的时间
    last_linear_acceleration_time_ 上个加速度时间
    orientation_ 姿态四元数
    gravity_vector_ 重力向量
    imu_angular_velocity_ 角速度
*/ 
ImuTracker::ImuTracker(const double imu_gravity_time_constant,
                       const common::Time time)
    : imu_gravity_time_constant_(imu_gravity_time_constant),
      time_(time),
      last_linear_acceleration_time_(common::Time::min()),
      orientation_(Eigen::Quaterniond::Identity()),
      gravity_vector_(Eigen::Vector3d::UnitZ()),
      imu_angular_velocity_(Eigen::Vector3d::Zero()) {}

// 更新姿态　orientation_　重力方向　gravity_vector_
void ImuTracker::Advance(const common::Time time) {
  CHECK_LE(time_, time);
  const double delta_t = common::ToSeconds(time - time_);

  // 返回旋转的四元数
  const Eigen::Quaterniond rotation =
      transform::AngleAxisVectorToRotationQuaternion(
          Eigen::Vector3d(imu_angular_velocity_ * delta_t));
  /*
    更新四元数,并对四元数进行归一化处理
    两个四元数的乘积,也表示旋转,也是一个四元数.
      orientation_ = q1 , rotation = q2
      q = orientation_ * rotation = q1 * q2
      将旋转后坐标系(O2)下的点P2,转换到原来坐标系(O)下P.
        P = q * p2 * q^-1 = (q1*q2)*p2*(p1*p2)^-1=q1 * (q2 * p2 * p2^-1) * p1^-1  
          其中:
            p1 = q2 * p2 * q2^-1   将O2坐标系下点,转换到O1坐标系下.
            P = q1 * p1 * p1^-1    将O1坐标系下的点,转换到O坐标系下
  */ 
  orientation_ = (orientation_ * rotation).normalized();

  /*
    反向旋转重力向量(向量不变,坐标系变化,故反向旋转)
      rotation.conjugate() 求共轭(反向旋转) rotation.inverse()求逆矩阵. q * q* = q * q^-1 = 1 四元数共轭矩阵等于其逆矩阵
      rotation.conjugate() * gravity_vector_
      v' = q * v = q * v *q^-1
  */ 
  gravity_vector_ = rotation.conjugate() * gravity_vector_;
  time_ = time;
}

/*
  IMU加速度值,更新重力方向和姿态四元数
*/ 
void ImuTracker::AddImuLinearAccelerationObservation(
    const Eigen::Vector3d& imu_linear_acceleration) {
  // Update the 'gravity_vector_' with an exponential moving average using the
  // 'imu_gravity_time_constant'.
  const double delta_t =
      last_linear_acceleration_time_ > common::Time::min()
          ? common::ToSeconds(time_ - last_linear_acceleration_time_)
          : std::numeric_limits<double>::infinity();
  last_linear_acceleration_time_ = time_;
  
  /*
      alpha = 1-exp(-t/10)
      gravity_vector_ = exp(-t/10) * gravity_vector_ + (1-exp(-t/10)) * imu_linear_acceleration
        t = 0,  gravity_vector_ = gravity_vector_
        t = +无穷,  gravity_vector_ = imu_linear_acceleration
        距离上一次更新时间越长,越相信加速度计数据,时间长累计误差大;
  */ 
  const double alpha = 1. - std::exp(-delta_t / imu_gravity_time_constant_); 
  gravity_vector_ =
      (1. - alpha) * gravity_vector_ + alpha * imu_linear_acceleration;
  // Change the 'orientation_' so that it agrees with the current
  // 'gravity_vector_'.

  /*
    计算重力, 在加速度校正前姿态下的方向向量: b = orientation_.conjugate() * Eigen::Vector3d::UnitZ()
    重力在校正后姿态下的方向向量: a = gravity_vector_
    Rb_a = FromTwoVectors(a,b) = rotation
    旋转四元数更新: orientation_ * rotation = Rg_b * Rb_a = Rg_a 得到从惯性系到a姿态下的变换四元数
  */ 
  const Eigen::Quaterniond rotation = FromTwoVectors(
      gravity_vector_, orientation_.conjugate() * Eigen::Vector3d::UnitZ());
  orientation_ = (orientation_ * rotation).normalized();
  CHECK_GT((orientation_ * gravity_vector_).z(), 0.);
  CHECK_GT((orientation_ * gravity_vector_).normalized().z(), 0.99);
}

// 角速度赋值
void ImuTracker::AddImuAngularVelocityObservation(
    const Eigen::Vector3d& imu_angular_velocity) {
  imu_angular_velocity_ = imu_angular_velocity;
}

}  // namespace mapping
}  // namespace cartographer
