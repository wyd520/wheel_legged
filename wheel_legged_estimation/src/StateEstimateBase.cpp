//
// Created by qiayuan on 2021/11/15.
//
#include <pinocchio/fwd.hpp>
#include "wheel_legged_estimation/StateEstimateBase.h"

#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <wheel_legged_interface/common/Types.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>

#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>

namespace ocs2{
namespace wheel_legged {

feet_array_t<std::vector<bool>> extractContactFlags(const std::vector<size_t>& phaseIDsStock)
{
  const size_t numPhases = phaseIDsStock.size();

  feet_array_t<std::vector<bool>> contactFlagStock;
  std::fill(contactFlagStock.begin(), contactFlagStock.end(), std::vector<bool>(numPhases));

  for (size_t i = 0; i < numPhases; i++)
  {
    const auto contactFlag = modeNumber2StanceLeg(phaseIDsStock[i]);
    for (size_t j = 0; j < contactFlag.size(); j++)
    {
      contactFlagStock[j][i] = contactFlag[j];
    }
  }
  return contactFlagStock;
}

StateEstimateBase::StateEstimateBase(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                                     const PinocchioEndEffectorKinematics& eeKinematics)
    : pinocchioInterface_(std::move(pinocchioInterface)), 
      info_(std::move(info)), 
      eeKinematics_(eeKinematics.clone()), 
      rbdState_(vector_t ::Zero(2 * info_.generalizedCoordinatesNum)), 
      latestStanceposition_{}{ 
  ros::NodeHandle nh; 
  odomPub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(nh, "odom", 10)); // 创建odomPub_，发布odom话题，频率为10Hz

  posePub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::PoseWithCovarianceStamped>(nh, "pose", 10)); // 创建posePub_，发布pose话题，频率为10Hz

  pSCgZinvlast_.resize(info_.generalizedCoordinatesNum);
  pSCgZinvlast_.setZero();

  estContactforce_.resize(16);
  estContactforce_.fill(50);

  vMeasuredLast_.resize(info_.generalizedCoordinatesNum);
  vMeasuredLast_.fill(0);

  cmdTorque_.resize(info_.actuatedDofNum);
  cmdTorque_.setZero();

  earlyLateContactMsg_.data.resize(4, 0);
}

// 更新关节状态
void StateEstimateBase::updateJointStates(const vector_t& jointPos, const vector_t& jointVel) {
  rbdState_.segment(6, info_.actuatedDofNum) = jointPos;
  rbdState_.segment(6 + info_.generalizedCoordinatesNum, info_.actuatedDofNum) = jointVel;
}

// 更新IMU状态
void StateEstimateBase::updateImu(const Eigen::Quaternion<scalar_t>& quat, const vector3_t& angularVelLocal,
                                  const vector3_t& linearAccelLocal, const matrix3_t& orientationCovariance,
                                  const matrix3_t& angularVelCovariance, const matrix3_t& linearAccelCovariance) {
  quat_ = quat;
  angularVelLocal_ = angularVelLocal;
  linearAccelLocal_ = linearAccelLocal;
  orientationCovariance_ = orientationCovariance;
  angularVelCovariance_ = angularVelCovariance;
  linearAccelCovariance_ = linearAccelCovariance;

  zyxOffset_(0) = imuBiasYaw_;
  zyxOffset_(1) = imuBiasPitch_;
  zyxOffset_(2) = imuBiasRoll_;

  vector3_t zyx = quatToZyx(quat) - zyxOffset_;
  vector3_t angularVelGlobal = getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(
      zyx, getEulerAnglesZyxDerivativesFromLocalAngularVelocity<scalar_t>(quatToZyx(quat), angularVelLocal));
  updateAngular(zyx, angularVelGlobal);
}

void StateEstimateBase::updateAngular(const vector3_t& zyx, const vector_t& angularVel) {
  rbdState_.segment<3>(0) = zyx;
  rbdState_.segment<3>(info_.generalizedCoordinatesNum) = angularVel;
}

void StateEstimateBase::updateLinear(const vector_t& pos, const vector_t& linearVel) {
  rbdState_.segment<3>(3) = pos;
  rbdState_.segment<3>(info_.generalizedCoordinatesNum + 3) = linearVel;
}

void StateEstimateBase::publishMsgs(const nav_msgs::Odometry& odom) {
  ros::Time time = odom.header.stamp;
  scalar_t publishRate = 200;
  if (lastPub_ + ros::Duration(1. / publishRate) < time) {
    lastPub_ = time;
    if (odomPub_->trylock()) {
      odomPub_->msg_ = odom;
      odomPub_->unlockAndPublish();
    }
    if (posePub_->trylock()) {
      posePub_->msg_.header = odom.header;
      posePub_->msg_.pose = odom.pose;
      posePub_->unlockAndPublish();
    }
  }
}

// void StateEstimateBase::setModeSchedule(const ocs2::ModeSchedule& schedule) {
//   modeSchedule_ = schedule;
// }

void StateEstimateBase::estContactForce(const ros::Duration& period) {
  scalar_t dt = period.toSec();
  if (dt <= 0 || dt > 1.0) dt = 0.002;

  // 一阶滤波参数（保持原样）
  const scalar_t lambda = cutoffFrequency_;
  const scalar_t gamma  = std::exp(-lambda * dt);
  const scalar_t beta   = (1 - gamma) / (gamma * dt);

  // 1) 重建 q, v
  vector_t qMeas(info_.generalizedCoordinatesNum), vMeas(info_.generalizedCoordinatesNum);
  // ── 与你原来是一样的──
  qMeas.head<3>() = rbdState_.segment<3>(3);
  qMeas.segment<3>(3) = rbdState_.head<3>();
  qMeas.tail(info_.actuatedDofNum) = rbdState_.segment(6, info_.actuatedDofNum);
  vMeas.head<3>() = rbdState_.segment<3>(info_.generalizedCoordinatesNum + 3);
  vMeas.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
      qMeas.segment<3>(3), rbdState_.segment<3>(info_.generalizedCoordinatesNum));
  vMeas.tail(info_.actuatedDofNum) = rbdState_.segment(info_.generalizedCoordinatesNum + 6, info_.actuatedDofNum);

  const auto& tauCmd = cmdTorque_;

  // 2) Pinocchio 更新，计算 M, C, g
  const auto& model = pinocchioInterface_.getModel();
  auto& data       = pinocchioInterface_.getData();

  pinocchio::forwardKinematics(model, data, qMeas, vMeas);
  pinocchio::computeJointJacobians(model, data);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::crba(model, data, qMeas);
  // 对称化 M
  data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  // pinocchio::computeCoriolisMatrix(model, data);
  pinocchio::nonLinearEffects(model, data, qMeas, vMeas);
  pinocchio::computeGeneralizedGravity(model, data, qMeas);

  // 3) 构造 pSCg = M*v_dot + C^T v + S^T τ - g
  //    这里 v_dot 用 beta*p + γ*last; p = M * vMeas
  matrix_t s(info_.actuatedDofNum, info_.generalizedCoordinatesNum);
  s.block(0, 0, info_.actuatedDofNum, 6).setZero();
  s.block(0, 6, info_.actuatedDofNum, info_.actuatedDofNum).setIdentity();
  vector_t coriolisTerm = data.nle - data.g;
  vector_t p = data.M * vMeas;
  vector_t pSCg = beta * p + /* slack：*/ s.transpose() * tauCmd 
                  + coriolisTerm - data.g;

  // 一阶滤波
  pSCgZinvlast_ = gamma * pSCgZinvlast_ + (1 - gamma) * pSCg;
  vector_t disturbance = beta * p - pSCgZinvlast_;

  // 4) 对每一条腿求接触力 λ_i ∈ R³
  const size_t nC = info_.numThreeDofContacts;
  estContactforce_.setZero(3 * nC);

  for (size_t i = 0; i < nC; ++i) {
    // 4.1 取出脚 i 的雅可比 Ji ∈ R^{3×nq}
    Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> J6; J6.setZero(6, info_.generalizedCoordinatesNum);
    pinocchio::getFrameJacobian(model, data, info_.endEffectorFrameIndices[i],
                                pinocchio::LOCAL_WORLD_ALIGNED, J6);
    Eigen::Matrix<scalar_t, 3, Eigen::Dynamic> Ji = J6.topRows<3>();

    // 4.2 构造局部方程： Ji * M⁻¹ * Jiᵀ * λ_i = Ji * M⁻¹ * disturbance
    //     先算 A = Ji * M⁻¹ * Jiᵀ  （3×3），  b = Ji * M⁻¹ * disturbance （3×1）
    Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic> Minv = data.M.inverse();
    Eigen::Matrix<scalar_t, 3, 3> A = Ji * Minv * Ji.transpose();
    Eigen::Matrix<scalar_t, 3, 1> b = Ji * Minv * disturbance;

    // 4.3 解线性方程 A λ = b，若怕数值病态可以加 1e-6 λ项正则化
    A.diagonal().array() += 1e-6;
    Eigen::Matrix<scalar_t, 3, 1> lambda_i = A.ldlt().solve(b);

    // 4.4 存起来
    estContactforce_.segment<3>(3 * i) = lambda_i;
  }

  for (int i = 0; i < 4; i++)
  {
    estContactforce_(6 * 2 + i) = estContactforce_.segment<3>(3 * i).norm();
    // std::cout << "leg " << i << " contact force: " << estContactforce_(6 * 2 + i) << std::endl;
  }
}

// contact_flag_t StateEstimateBase::estContactState(const scalar_t& time) {
//   // 1) 拿到整个 ModeSchedule
//   const auto& schedule     = modeSchedule_;
//   const auto& phaseIDs      = schedule.modeSequence;       // vector<size_t>，每个 phase 对应的 modeNumber
//   const auto& switchTimes   = schedule.switchingTimes; // vector<scalar_t>，长度 = phaseIDs.size()+1
  
//   // 2) 找到当前 time 落在哪个 phase
//   //    switchTimes 定义为：[ t0, t1, t2, ..., tN ], 其中 t0=0, tN=总 horizon
//   //    phase i 有效区间 [ t_i, t_{i+1} )
//   // 3. 在 switchingTs 中找出当前 time 落在哪个 interval [ t_k, t_{k+1} )
//   size_t k = 0;
//   // 如果 currentTime 在最后一个切换点之后，就选最后一个 phase
//   while (k + 1 < switchingTs.size() && currentTime >= switchingTs[k + 1]) {
//     ++k;
//   }
//   // k 现在是对应的 phase 索引

//   // 4. 取出这一 phase 的 modeNumber
//   size_t mode = phaseIDs[k];

//   // 5. 将 modeNumber 解码成四足上 4 个足端的 contact_flag_t（bool[4]）
//   contact_flag_t contactFlag = modeNumber2StanceLeg(mode);

//   return contactFlag;
// }

contact_flag_t StateEstimateBase::estContactState(const scalar_t& time) {
    // 1. 根据 time 从 modeSchedule_ 中查找当前模式号
    size_t modeNumber = modeSchedule_.modeAtTime(time); 
    // 2. 得到参考支撑腿标志（理论支撑腿组合）
    contact_flag_t stanceLegRef = modeNumber2StanceLeg(modeNumber);
    
    // 3. 基于阈值判断实际接触腿
    //    estContactforce_ 的第 12~15 项分别为四条腿的接触力大小
    contact_flag_t contactFlags = cmdContactflag_;  // 初始化所有标志为 false
    for (size_t i = 0; i < 4; ++i) {
        // 参考支撑腿可用作参考（此处仅阈值判断实际接触）
        bool isContact = (estContactforce_(12 + i) > contactThreshold_);
        contactFlags[i] = isContact;
    }
    
    // 4. 返回实际接触标志列表
    return std::move(contactFlags);
}

}  // namespace legged
}