//
// Created by qiayuan on 2022/6/24.
//

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "wheel_legged_controllers/WheelLeggedController.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <wheel_legged_dummy/gait/GaitReceiver.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_sqp/SqpMpc.h>

#include <angles/angles.h>
#include <wheel_legged_estimation/FromTopiceEstimate.h>
#include <wheel_legged_estimation/LinearKalmanFilter.h>
#include <wheel_legged_wbc/WeightedWbc.h>
#include <pluginlib/class_list_macros.hpp>

namespace wheel_legged_controller {
using namespace ocs2;
using namespace wheel_legged;
bool WheelLeggedController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) {
  // Initialize OCS2
  
  std::string urdfFile;
  std::string taskFile;
  std::string referenceFile;
  controller_nh.getParam("/urdfFile", urdfFile);
  controller_nh.getParam("/taskFile", taskFile);
  controller_nh.getParam("/referenceFile", referenceFile);
  
  bool verbose = false;
  
  loadData::loadCppDataType(taskFile, "Wheel_legged_robot.verbose", verbose);
  
  setupLeggedInterface(taskFile, urdfFile, referenceFile, verbose);
  setupMpc();
  setupMrt();
  
  // Visualization
  ros::NodeHandle nh;
  CentroidalModelPinocchioMapping pinocchioMapping(WheelleggedInterface_->getCentroidalModelInfo());
  
  eeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(WheelleggedInterface_->getPinocchioInterface(), pinocchioMapping,
                                                                      WheelleggedInterface_->modelSettings().contactNames3DoF);
  robotVisualizer_ = std::make_shared<WheelLeggedRobotVisualizer>(WheelleggedInterface_->getPinocchioInterface(),
                                                             WheelleggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_, nh);
// 获取几何接口指针
  // PinocchioGeometryInterface& geometryInterface = WheelleggedInterface_->getGeometryInterface();
  // if (WheelleggedInterface_->getGeometryInterface() == nullptr) {
  //   ROS_ERROR("GeometryInterface is null!");
  //   return;  // 或抛出异常
  // }

  // selfCollisionVisualization_.reset(new LeggedSelfCollisionVisualization(WheelleggedInterface_->getPinocchioInterface(),
  //                                                                        WheelleggedInterface_->getGeometryInterface(), pinocchioMapping, nh));
  // rbdConversions_ = std::make_shared<CentroidalModelRbdConversions>(WheelleggedInterface_->getPinocchioInterface(),
  //                                                                   WheelleggedInterface_->getCentroidalModelInfo());
  
  defalutJointPos_.resize(jointNum_);
  loadData::loadEigenMatrix(referenceFile, "defaultJointState", defalutJointPos_);

  // Hardware interface
  // auto* hybridJointInterface = robot_hw->get<HybridJointInterface>();

  // std::vector<std::string> joint_names{"Joint_LBhip", "Joint_LBknee", "Joint_LWheel", "Joint_RBhip", "Joint_RBknee", "Joint_RWheel",
  //                                      "Joint_Labad", "Joint_Lhip", "Joint_Lknee", "Joint_Rabad", "Joint_Rhip", "Joint_Rknee"};//修改为自己的urdf中的关节
  // for (const auto& joint_name : joint_names) {
  //   hybridJointHandles_.push_back(hybridJointInterface->getHandle(joint_name));
  // }
  // auto* contactInterface = robot_hw->get<ContactSensorInterface>();
  // for (const auto& name : WheelleggedInterface_->modelSettings().contactNames3DoF) {
  //   contactHandles_.push_back(contactInterface->getHandle(name));
  // }
  // imuSensorHandle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("base_imu");

  jointPos_ = vector_t::Zero(jointNum_);
  jointPos_ << 0.0, -0.07, 0.0, -0.07, 0, 0.913, -1.85, 0.0, 0.913, -1.85;
  jointVel_ = vector_t::Zero(jointNum_);
  quat_ = Eigen::Quaternion<scalar_t>(1, 0, 0, 0);
  jointPosVelSub_ =  controller_nh.subscribe<std_msgs::Float32MultiArray>("/jointsPosVel", 10, &WheelLeggedController::jointStateCallback, this);
  imuSub_ = controller_nh.subscribe<sensor_msgs::Imu>("/base_imu", 10, &WheelLeggedController::ImuCallback, this);
  targetTorquePub_ = controller_nh.advertise<std_msgs::Float32MultiArray>("/targetTorque", 10);
  targetPosPub_ = controller_nh.advertise<std_msgs::Float32MultiArray>("/targetPos", 10);
  targetVelPub_ = controller_nh.advertise<std_msgs::Float32MultiArray>("/targetVel", 10);
  targetKpPub_ = controller_nh.advertise<std_msgs::Float32MultiArray>("/targetKp", 10);
  targetKdPub_ = controller_nh.advertise<std_msgs::Float32MultiArray>("/targetKd", 10);


  // State estimation
  setupStateEstimate(taskFile, verbose);

  // Whole body control
  wbc_ = std::make_shared<WeightedWbc>(WheelleggedInterface_->getPinocchioInterface(), 
                                       WheelleggedInterface_->getCentroidalModelInfo(),
                                       *eeKinematicsPtr_);
  wbc_->loadTasksSetting(taskFile, verbose);

  // Safety Checker
  safetyChecker_ = std::make_shared<SafetyChecker>(WheelleggedInterface_->getCentroidalModelInfo());
  
  // eeKinematicsPtr_->setPinocchioInterface(WheelleggedInterface_->getPinocchioInterface());
  // RetrievingParameters();

  // inverseKinematics_.setParam(std::make_shared<PinocchioInterface>(WheelleggedInterface_->getPinocchioInterface()),
  //                             std::make_shared<CentroidalModelInfo>(WheelleggedInterface_->getCentroidalModelInfo()));

  return true;
}

void WheelLeggedController::jointStateCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  if (msg->data.size() != 2 * jointNum_) {
    ROS_ERROR_STREAM("Received joint state message with wrong size: " << msg->data.size());
    return;
  }
  for (size_t i = 0; i < jointNum_; ++i) {
      jointPos_(i) = msg->data[i];
      jointVel_(i) = msg->data[i + jointNum_];
  }
}

void WheelLeggedController::ImuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
    quat_.coeffs().w() = msg->orientation.w;
    quat_.coeffs().x() = msg->orientation.x;
    quat_.coeffs().y() = msg->orientation.y;
    quat_.coeffs().z() = msg->orientation.z;
    angularVel_ << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
    linearAccel_ << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
    orientationCovariance_ << msg->orientation_covariance[0], msg->orientation_covariance[1], msg->orientation_covariance[2],
            msg->orientation_covariance[3], msg->orientation_covariance[4], msg->orientation_covariance[5],
            msg->orientation_covariance[6], msg->orientation_covariance[7], msg->orientation_covariance[8];
    angularVelCovariance_ << msg->angular_velocity_covariance[0], msg->angular_velocity_covariance[1], msg->angular_velocity_covariance[2],
            msg->angular_velocity_covariance[3], msg->angular_velocity_covariance[4], msg->angular_velocity_covariance[5],
            msg->angular_velocity_covariance[6], msg->angular_velocity_covariance[7], msg->angular_velocity_covariance[8];
    linearAccelCovariance_ << msg->linear_acceleration_covariance[0], msg->linear_acceleration_covariance[1], msg->linear_acceleration_covariance[2],
            msg->linear_acceleration_covariance[3], msg->linear_acceleration_covariance[4], msg->linear_acceleration_covariance[5],
            msg->linear_acceleration_covariance[6], msg->linear_acceleration_covariance[7], msg->linear_acceleration_covariance[8];
}

void WheelLeggedController::starting(const ros::Time& time) {
  // Initial state,还需要加入关节状态
  
  // startingTime_.fromSec(time.toSec() - 0.0001);
  // const ros::Time shifted_time = time - startingTime_;

  currentObservation_.state.setZero(WheelleggedInterface_->getCentroidalModelInfo().stateDim);
  currentObservation_.state(8) = 0.25;
  currentObservation_.state.segment(6 + 6, jointNum_) = defalutJointPos_;

  updateStateEstimation(time, ros::Duration(0.002));
  currentObservation_.input.setZero(WheelleggedInterface_->getCentroidalModelInfo().inputDim);
  currentObservation_.mode = ModeNumber::STANCE;

  TargetTrajectories target_trajectories({currentObservation_.time}, 
                                         {currentObservation_.state}, 
                                         {currentObservation_.input});

  // Set the first observation and command and wait for optimization to finish
  mpcMrtInterface_->setCurrentObservation(currentObservation_);
  mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target_trajectories);
  ROS_INFO_STREAM("Waiting for the initial policy ...");
  while (!mpcMrtInterface_->initialPolicyReceived() && ros::ok()) {
    mpcMrtInterface_->advanceMpc();
    ros::WallRate(WheelleggedInterface_->mpcSettings().mrtDesiredFrequency_).sleep();
  }
  ROS_INFO_STREAM("Initial policy has been received.");
  // ModeSubscribe();
  mpcRunning_ = true;
}

void WheelLeggedController::update(const ros::Time& time, const ros::Duration& period) {
  // State Estimate
  updateStateEstimation(time, period);

  // Update the current state of the system
  mpcMrtInterface_->setCurrentObservation(currentObservation_);

  // Load the latest MPC policy
  mpcMrtInterface_->updatePolicy();

  // Evaluate the current policy
  vector_t optimizedState, optimizedInput;
  // size_t plannedMode = 0;  // The mode that is active at the time the policy is evaluated at.
  mpcMrtInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState, optimizedInput, plannedMode_);

  // Whole body control
  currentObservation_.input = optimizedInput;

  wbcTimer_.startTimer();
  vector_t x = wbc_->update(optimizedState, optimizedInput, measuredRbdState_, plannedMode_, period.toSec());
  wbcTimer_.endTimer();

  const vector_t& torque = x.tail(jointNum_);
  const vector_t& wbc_planned_joint_acc = x.segment(6, jointNum_);
  const vector_t& wbc_planned_body_acc = x.head(6);
  const vector_t& wbc_planned_contact_force = x.segment(6 + jointNum_, wbc_->getContactForceSize());

  vector_t posDes = centroidal_model::getJointAngles(optimizedState, WheelleggedInterface_->getCentroidalModelInfo());
  vector_t velDes = centroidal_model::getJointVelocities(optimizedInput, WheelleggedInterface_->getCentroidalModelInfo());

  scalar_t dt = period.toSec();
  posDes = posDes + 0.5 * wbc_planned_joint_acc * dt * dt;
  velDes = velDes + wbc_planned_joint_acc * dt;
  // printf("posDes: %f %f %f %f %f %f %f %f %f %f\n", posDes(0), posDes(1), posDes(2), posDes(3), posDes(4), posDes(5), posDes(6), posDes(7), posDes(8), posDes(9));
  // printf("velDes: %f %f %f %f %f %f %f %f %f %f\n", velDes(0), velDes(1), velDes(2), velDes(3), velDes(4), velDes(5), velDes(6), velDes(7), velDes(8), velDes(9));
  
  // Safety check, if failed, stop the controller
  if (!safetyChecker_->check(currentObservation_, optimizedState, optimizedInput)) {
    ROS_ERROR_STREAM("[Legged Controller] Safety check failed, stopping the controller.");
    stopRequest(time);
  }

  std_msgs::Float32MultiArray targetTorqueMsg;
    for (int i1 = 0; i1 < 10; ++i1) {
        targetTorqueMsg.data.push_back(torque(i1));
    }
    std_msgs::Float32MultiArray targetPosMsg;
    for (int i1 = 0; i1 < 10; ++i1) {
        targetPosMsg.data.push_back(posDes(i1));
    }
    std_msgs::Float32MultiArray targetVelMsg;
    for (int i1 = 0; i1 < 10; ++i1) {
        targetVelMsg.data.push_back(velDes(i1));
    }
    targetTorquePub_.publish(targetTorqueMsg);
    targetPosPub_.publish(targetPosMsg);
    targetVelPub_.publish(targetVelMsg);
    std_msgs::Float32MultiArray targetKp;
    std_msgs::Float32MultiArray targetKd;

    targetKp.data = {90, 90,
                     90, 90,
                     60, 60, 60,
                     60, 60, 60};
    targetKd.data = {1.2, 1.2,
                     1.2, 1.2,
                     0.9, 0.9, 0.9,
                     0.9, 0.9, 0.9};

   
    targetKpPub_.publish(targetKp);
    targetKdPub_.publish(targetKd);

  // Visualization
  robotVisualizer_->update(currentObservation_, mpcMrtInterface_->getPolicy(), mpcMrtInterface_->getCommand());
  // selfCollisionVisualization_->update(currentObservation_);

  // Publish the observation. Only needed for the command interface
  observationPublisher_.publish(ros_msg_conversions::createObservationMsg(currentObservation_));
}

void WheelLeggedController::updateStateEstimation(const ros::Time& time, const ros::Duration& period) {
  vector_t jointPos(jointNum_), jointVel(jointNum_);
  contact_flag_t contacts;
  Eigen::Quaternion<scalar_t> quat;
  contact_flag_t contactFlag;
  vector3_t angularVel, linearAccel;
  matrix3_t orientationCovariance, angularVelCovariance, linearAccelCovariance;

  jointPos = jointPos_;
  jointVel = jointVel_;
  contactFlag = modeNumber2StanceLeg(plannedMode_);

  quat = quat_;
  angularVel = angularVel_;
  linearAccel = linearAccel_;
  orientationCovariance = orientationCovariance_;
  angularVelCovariance = angularVelCovariance_;
  linearAccelCovariance = linearAccelCovariance_;

  stateEstimate_->updateJointStates(jointPos, jointVel);
  stateEstimate_->updateContact(contactFlag);
  stateEstimate_->updateImu(quat, angularVel, linearAccel, orientationCovariance, angularVelCovariance, linearAccelCovariance);
  measuredRbdState_ = stateEstimate_->update(time, period);
  currentObservation_.time += period.toSec();
  scalar_t yawLast = currentObservation_.state(9);
  currentObservation_.state = rbdConversions_->computeCentroidalStateFromRbdModel(measuredRbdState_);
  currentObservation_.state(9) = yawLast + angles::shortest_angular_distance(yawLast, currentObservation_.state(9));
  currentObservation_.mode = plannedMode_;
  // currentObservation_.mode = stateEstimate_->getMode();
}

WheelLeggedController::~WheelLeggedController() {
  controllerRunning_ = false;
  if (mpcThread_.joinable()) {
    mpcThread_.join();
  }
  std::cerr << "########################################################################";
  std::cerr << "\n### MPC Benchmarking";
  std::cerr << "\n###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cerr << "\n###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms]." << std::endl;
  std::cerr << "########################################################################";
  std::cerr << "\n### WBC Benchmarking";
  std::cerr << "\n###   Maximum : " << wbcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cerr << "\n###   Average : " << wbcTimer_.getAverageInMilliseconds() << "[ms].";
}

void WheelLeggedController::setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                            bool verbose) {
  WheelleggedInterface_ = std::make_shared<WheelLeggedInterface>(taskFile, urdfFile, referenceFile);
  WheelleggedInterface_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);
}

void WheelLeggedController::setupMpc() {
  mpc_ = std::make_shared<SqpMpc>(WheelleggedInterface_->mpcSettings(), WheelleggedInterface_->sqpSettings(),
                                  WheelleggedInterface_->getOptimalControlProblem(), WheelleggedInterface_->getInitializer());
  rbdConversions_ = std::make_shared<CentroidalModelRbdConversions>(WheelleggedInterface_->getPinocchioInterface(),
                                                                    WheelleggedInterface_->getCentroidalModelInfo());

  const std::string robotName = "wheel_legged";
  ros::NodeHandle nh;
  // Gait receiver
  auto gaitReceiverPtr =
      std::make_shared<GaitReceiver>(nh, WheelleggedInterface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), robotName);
  // ROS ReferenceManager
  auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(robotName, WheelleggedInterface_->getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(nh);
  mpc_->getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);
  mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
  observationPublisher_ = nh.advertise<ocs2_msgs::mpc_observation>(robotName + "_mpc_observation", 1);
}

void WheelLeggedController::setupMrt() {
  mpcMrtInterface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
  mpcMrtInterface_->initRollout(&WheelleggedInterface_->getRollout());
  mpcTimer_.reset();

  // stateEstimate_->setModeSchedule(mpcMrtInterface_->getReferenceManager().getModeSchedule());

  controllerRunning_ = true;
  mpcThread_ = std::thread([&]() {
    while (controllerRunning_) {
      try {
        executeAndSleep(
            [&]() {
              if (mpcRunning_) {
                mpcTimer_.startTimer();
                mpcMrtInterface_->advanceMpc();
                mpcTimer_.endTimer();
              }
            },
            WheelleggedInterface_->mpcSettings().mpcDesiredFrequency_);
      } catch (const std::exception& e) {
        controllerRunning_ = false;
        ROS_ERROR_STREAM("[Ocs2 MPC thread] Error : " << e.what());
        stopRequest(ros::Time());
      }
    }
  });
  setThreadPriority(WheelleggedInterface_->sqpSettings().threadPriority, mpcThread_);
}

void WheelLeggedController::loadControllerCallback(const std_msgs::Float32::ConstPtr& msg)
{
  loadControllerFlag_ = true;
  mpcRunning_ = true;
  ROS_INFO("Successfully load the controller");
}

void WheelLeggedController::setupStateEstimate(const std::string& taskFile, bool verbose) {
  const auto modeSchedule = mpcMrtInterface_->getReferenceManager().getModeSchedule();
  stateEstimate_ = std::make_shared<KalmanFilterEstimate>(WheelleggedInterface_->getPinocchioInterface(),
                                                          WheelleggedInterface_->getCentroidalModelInfo(), 
                                                          *eeKinematicsPtr_);
  // stateEstimate_->loadSettings(taskFile, verbose);
  dynamic_cast<KalmanFilterEstimate&>(*stateEstimate_).loadSettings(taskFile, verbose);
  currentObservation_.time = 0;
}

void WheelLeggedCheaterController::setupStateEstimate(const std::string& /*taskFile*/, bool /*verbose*/) {
  const auto modeSchedule = mpcMrtInterface_->getReferenceManager().getModeSchedule();
  stateEstimate_ = std::make_shared<FromTopicStateEstimate>(WheelleggedInterface_->getPinocchioInterface(),
                                                            WheelleggedInterface_->getCentroidalModelInfo(), 
                                                            *eeKinematicsPtr_, controllerNh_);
}

}  // namespace legged

PLUGINLIB_EXPORT_CLASS(wheel_legged_controller::WheelLeggedController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(wheel_legged_controller::WheelLeggedCheaterController, controller_interface::ControllerBase)
