//
// Created by qiayuan on 2022/6/24.
//

#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <wheel_legged_common/hardware_interface/ContactSensorInterface.h>

#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/misc/Benchmark.h>
#include <wheel_legged_dummy/visualization/WheelLeggedRobotVisualizer.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>

#include <wheel_legged_estimation/StateEstimateBase.h>
#include <wheel_legged_interface/LeggedInterface.h>
#include <wheel_legged_wbc/WbcBase.h>

#include "wheel_legged_controllers/SafetyChecker.h"
#include "wheel_legged_controllers/visualization/LeggedSelfCollisionVisualization.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float32.h"

namespace wheel_legged_controller {
using namespace ocs2;
using namespace wheel_legged;
class WheelLeggedController : public controller_interface::MultiInterfaceController<HybridJointInterface, hardware_interface::ImuSensorInterface,
                                                                               ContactSensorInterface> {
 public:
  WheelLeggedController() = default;
  ~WheelLeggedController() override;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void starting(const ros::Time& time) override;
  void stopping(const ros::Time& /*time*/) override { mpcRunning_ = false; }

 protected:
  virtual void updateStateEstimation(const ros::Time& time, const ros::Duration& period);

  virtual void setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                    bool verbose);
  virtual void setupMpc();
  virtual void setupMrt();
  virtual void setupStateEstimate(const std::string& taskFile, bool verbose);

  void RetrievingParameters();
  void jointStateCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
  void ImuCallback(const sensor_msgs::Imu::ConstPtr& msg);

  void loadControllerCallback(const std_msgs::Float32::ConstPtr& msg);

  void ModeSubscribe();
  

  // Interface
  std::shared_ptr<WheelLeggedInterface> WheelleggedInterface_;
  std::shared_ptr<PinocchioEndEffectorKinematics> eeKinematicsPtr_;
  std::vector<HybridJointHandle> hybridJointHandles_;
  std::vector<ContactSensorHandle> contactHandles_;
  hardware_interface::ImuSensorHandle imuSensorHandle_;

  // State Estimation
  SystemObservation currentObservation_;
  vector_t measuredRbdState_;
  std::shared_ptr<StateEstimateBase> stateEstimate_;
  std::shared_ptr<CentroidalModelRbdConversions> rbdConversions_;

  // Whole Body Control
  std::shared_ptr<WbcBase> wbc_;
  std::shared_ptr<SafetyChecker> safetyChecker_;

  // Nonlinear MPC
  std::shared_ptr<MPC_BASE> mpc_;
  std::shared_ptr<MPC_MRT_Interface> mpcMrtInterface_;

  // Visualization
  std::shared_ptr<WheelLeggedRobotVisualizer> robotVisualizer_;
  std::shared_ptr<ocs2::wheel_legged::LeggedSelfCollisionVisualization> selfCollisionVisualization_;
  ros::Publisher observationPublisher_;

  //Controller Interface
  ros::Publisher targetTorquePub_;
  ros::Publisher targetPosPub_;
  ros::Publisher targetVelPub_;
  ros::Publisher targetKpPub_;
  ros::Publisher targetKdPub_;
  ros::Subscriber jointPosVelSub_;
  ros::Subscriber imuSub_;
  ros::Subscriber subLoadcontroller_;

  // Node Handle
  ros::NodeHandle controllerNh_;

 private:
  std::thread mpcThread_;
  std::atomic_bool controllerRunning_{}, mpcRunning_{};
  benchmark::RepeatedTimer mpcTimer_;
  benchmark::RepeatedTimer wbcTimer_;
  size_t jointNum_ = 10;//这个是假定的，需要根据实际情况修改
  vector_t jointPos_, jointVel_;
  Eigen::Quaternion<scalar_t> quat_;
  contact_flag_t contactFlag_;
  vector3_t angularVel_, linearAccel_;
  matrix3_t orientationCovariance_, angularVelCovariance_, linearAccelCovariance_;
  size_t plannedMode_ = 11;
  vector_t defalutJointPos_;
  bool loadControllerFlag_{ false };
  // InverseKinematics inverseKinematics_;
};

class WheelLeggedCheaterController : public WheelLeggedController {
 protected:
  void setupStateEstimate(const std::string& taskFile, bool verbose) override;
};

}  // namespace legged
