//
// Created by qiayuan on 2022/7/24.
//

#include "wheel_legged_estimation/FromTopiceEstimate.h"
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>

namespace ocs2{
namespace wheel_legged {

FromTopicStateEstimate::FromTopicStateEstimate(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                                               const PinocchioEndEffectorKinematics& eeKinematics, ::ros::NodeHandle& nh)
    : StateEstimateBase(std::move(pinocchioInterface), std::move(info), eeKinematics) 
{
  sub_ = nh.subscribe<nav_msgs::Odometry>("/ground_truth/state", 10, &FromTopicStateEstimate::callback, this);
}

void FromTopicStateEstimate::callback(const nav_msgs::Odometry::ConstPtr& msg) {
  buffer_.writeFromNonRT(*msg);
}

// 从主题状态估计中更新
vector_t FromTopicStateEstimate::update(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
  nav_msgs::Odometry odom = *buffer_.readFromRT();

  Eigen::Quaternion<scalar_t> quat(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
                                    odom.pose.pose.orientation.z);
  auto angularVelLocal = Eigen::Matrix<scalar_t, 3, 1>(odom.twist.twist.angular.x, odom.twist.twist.angular.y,
                                                        odom.twist.twist.angular.z);
  vector3_t zyx = quatToZyx(quat) - zyxOffset_;
  vector3_t angularVelGlobal = getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(
      zyx, getEulerAnglesZyxDerivativesFromLocalAngularVelocity<scalar_t>(quatToZyx(quat), angularVelLocal));
  updateAngular(zyx, angularVelGlobal);

  updateLinear(Eigen::Matrix<scalar_t, 3, 1>(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z),
               Eigen::Matrix<scalar_t, 3, 1>(odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z));

  publishMsgs(odom);

  // 返回机器人状态
  return rbdState_;
}

}  // namespace legged
}
