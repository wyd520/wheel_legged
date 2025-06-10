/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

// Pinocchio forward declarations must be included first
#include <pinocchio/fwd.hpp>

// Pinocchio
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "wheel_legged_dummy/visualization/WheelLeggedRobotVisualizer.h"

// OCS2
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_ros_interfaces/visualization/VisualizationHelpers.h>
#include "wheel_legged_interface/gait/MotionPhaseDefinition.h"

// Additional messages not in the helpers file
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

// URDF related
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

namespace ocs2 {
namespace wheel_legged {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
// 构造函数，初始化LeggedRobotVisualizer对象
WheelLeggedRobotVisualizer::WheelLeggedRobotVisualizer(PinocchioInterface pinocchioInterface, CentroidalModelInfo centroidalModelInfo,
                                             const PinocchioEndEffectorKinematics& endEffectorKinematics, ros::NodeHandle& nodeHandle,
                                             scalar_t maxUpdateFrequency)
    : pinocchioInterface_(std::move(pinocchioInterface)), // 移动语义，将pinocchioInterface的所有权转移给pinocchioInterface_
      centroidalModelInfo_(std::move(centroidalModelInfo)), // 移动语义，将centroidalModelInfo的所有权转移给centroidalModelInfo_
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()), // 克隆endEffectorKinematics，并将指针赋值给endEffectorKinematicsPtr_
      lastTime_(std::numeric_limits<scalar_t>::lowest()), // 初始化lastTime_为最小值
      minPublishTimeDifference_(1.0 / maxUpdateFrequency) { // 计算最小发布时间差
  endEffectorKinematicsPtr_->setPinocchioInterface(pinocchioInterface_); // 设置endEffectorKinematicsPtr_的Pinocchio接口
  launchVisualizerNode(nodeHandle); // 启动可视化节点
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void WheelLeggedRobotVisualizer::launchVisualizerNode(ros::NodeHandle& nodeHandle) {
  // 发布期望的基座轨迹
  costDesiredBasePositionPublisher_ = nodeHandle.advertise<visualization_msgs::Marker>("/wheel_legged/desiredBaseTrajectory", 1);
  // 发布期望的脚部轨迹
  costDesiredFeetPositionPublishers_.resize(centroidalModelInfo_.numThreeDofContacts);
  costDesiredFeetPositionPublishers_[0] = nodeHandle.advertise<visualization_msgs::Marker>("/wheel_legged/desiredFeetTrajectory/LF", 1);
  costDesiredFeetPositionPublishers_[1] = nodeHandle.advertise<visualization_msgs::Marker>("/wheel_legged/desiredFeetTrajectory/RF", 1);
  costDesiredFeetPositionPublishers_[2] = nodeHandle.advertise<visualization_msgs::Marker>("/wheel_legged/desiredFeetTrajectory/LH", 1);
  costDesiredFeetPositionPublishers_[3] = nodeHandle.advertise<visualization_msgs::Marker>("/wheel_legged/desiredFeetTrajectory/RH", 1);
  // 发布优化后的状态轨迹
  stateOptimizedPublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("/wheel_legged/optimizedStateTrajectory", 1);
  // 发布当前状态
  currentStatePublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("/wheel_legged/currentState", 1);

  // Load URDF model
  urdf::Model urdfModel;
  if (!urdfModel.initParam("wheel_legged_description")) {
    std::cerr << "[WheelLeggedRobotVisualizer] Could not read URDF from: \"wheel_legged_description\"" << std::endl;
  } else {
    KDL::Tree kdlTree;
    kdl_parser::treeFromUrdfModel(urdfModel, kdlTree);

    robotStatePublisherPtr_.reset(new robot_state_publisher::RobotStatePublisher(kdlTree));
    robotStatePublisherPtr_->publishFixedTransforms(true);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void WheelLeggedRobotVisualizer::update(const SystemObservation& observation, const PrimalSolution& primalSolution, const CommandData& command) {
  // 如果当前时间与上次发布时间之间的差值大于最小发布时间差值，则进行更新
  if (observation.time - lastTime_ > minPublishTimeDifference_) {
    // 获取模型
    const auto& model = pinocchioInterface_.getModel();
    // 获取数据
    auto& data = pinocchioInterface_.getData();
    // 计算前向运动学
    pinocchio::forwardKinematics(model, data, centroidal_model::getGeneralizedCoordinates(observation.state, centroidalModelInfo_));
    // 更新帧位置
    pinocchio::updateFramePlacements(model, data);

    // 获取当前时间戳
    const auto timeStamp = ros::Time::now();
    // 发布观测
    publishObservation(timeStamp, observation);
    // 发布期望轨迹
    publishDesiredTrajectory(timeStamp, command.mpcTargetTrajectories_);
    // 发布优化后的状态轨迹
    publishOptimizedStateTrajectory(timeStamp, primalSolution.timeTrajectory_, primalSolution.stateTrajectory_,
                                    primalSolution.modeSchedule_);
    // 更新上次发布时间
    lastTime_ = observation.time;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void WheelLeggedRobotVisualizer::publishObservation(ros::Time timeStamp, const SystemObservation& observation) {
  // Extract components from state
  // 从状态中提取组件
  const auto basePose = centroidal_model::getBasePose(observation.state, centroidalModelInfo_);
  const auto qJoints = centroidal_model::getJointAngles(observation.state, centroidalModelInfo_);

  // Compute cartesian state and inputs
  // 计算笛卡尔状态和输入
  const auto feetPositions = endEffectorKinematicsPtr_->getPosition(observation.state);
  std::vector<vector3_t> feetForces(centroidalModelInfo_.numThreeDofContacts);
  for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
    feetForces[i] = centroidal_model::getContactForces(observation.input, i, centroidalModelInfo_);
  }

  // Publish
  // 发布
  publishJointTransforms(timeStamp, qJoints);
  publishBaseTransform(timeStamp, basePose);
  publishCartesianMarkers(timeStamp, modeNumber2StanceLeg(observation.mode), feetPositions, feetForces);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*概述：LeggedRobotVisualizer::publishJointTransforms 函数用于发布机器人的关节变换信息。
       它将关节角度转换为关节位置，并通过 ROS 发布这些变换信息。*/
void WheelLeggedRobotVisualizer::publishJointTransforms(ros::Time timeStamp, const vector_t& jointAngles) const {
  if (robotStatePublisherPtr_ != nullptr) {
    std::map<std::string, scalar_t> jointPositions{{"Joint_LBhip", jointAngles[0]}, {"Joint_LBknee", jointAngles[1]},  
                                                   {"Joint_RBhip", jointAngles[2]}, {"Joint_RBknee", jointAngles[3]},  
                                                   {"Joint_Labad", jointAngles[4]}, {"Joint_Lhip", jointAngles[5]},  {"Joint_Lknee", jointAngles[6]},
                                                   {"Joint_Rabad", jointAngles[7]}, {"Joint_Rhip", jointAngles[8]}, {"Joint_Rknee", jointAngles[9]}};
    robotStatePublisherPtr_->publishTransforms(jointPositions, timeStamp);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
// 发布机器人基座变换
void WheelLeggedRobotVisualizer::publishBaseTransform(ros::Time timeStamp, const vector_t& basePose) {
  // 如果机器人状态发布器不为空
  if (robotStatePublisherPtr_ != nullptr) {
    // 创建基座到世界坐标系的变换
    geometry_msgs::TransformStamped baseToWorldTransform;
    // 设置变换的头部信息
    baseToWorldTransform.header = getHeaderMsg(frameId_, timeStamp);
    // 设置变换的子坐标系
    baseToWorldTransform.child_frame_id = "dummy_link";

    // 从欧拉角获取基座到世界坐标系的四元数
    const Eigen::Quaternion<scalar_t> q_world_base = getQuaternionFromEulerAnglesZyx(vector3_t(basePose.tail<3>()));
    // 设置变换的旋转信息
    baseToWorldTransform.transform.rotation = getOrientationMsg(q_world_base);
    // 设置变换的平移信息
    baseToWorldTransform.transform.translation = getVectorMsg(basePose.head<3>());
    // 发送变换
    tfBroadcaster_.sendTransform(baseToWorldTransform);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
// 发布轨迹
void WheelLeggedRobotVisualizer::publishTrajectory(const std::vector<SystemObservation>& system_observation_array, scalar_t speed) {
  // 遍历系统观测数组
  for (size_t k = 0; k < system_observation_array.size() - 1; k++) {
    // 计算帧持续时间
    scalar_t frameDuration = speed * (system_observation_array[k + 1].time - system_observation_array[k].time);
    // 计算发布持续时间
    scalar_t publishDuration = timedExecutionInSeconds([&]() { publishObservation(ros::Time::now(), system_observation_array[k]); });
    // 如果帧持续时间大于发布持续时间，则休眠
    if (frameDuration > publishDuration) {
      ros::WallDuration(frameDuration - publishDuration).sleep();
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*概述：LeggedRobotVisualizer::publishCartesianMarkers 函数用于发布机器人的笛卡尔坐标标记，包括支撑多边形的标记和脚部位置及力标记。*/
void WheelLeggedRobotVisualizer::publishCartesianMarkers(ros::Time timeStamp, const contact_flag_t& contactFlags,
                                                    const std::vector<vector3_t>& feetPositions,
                                                    const std::vector<vector3_t>& feetForces) const {
  // Reserve message
  const size_t numberOfCartesianMarkers = 10;
  visualization_msgs::MarkerArray markerArray;
  markerArray.markers.reserve(numberOfCartesianMarkers);

  // Feet positions and Forces
  for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; ++i) {
    // Get foot marker
    markerArray.markers.emplace_back(
        getFootMarker(feetPositions[i], contactFlags[i], feetColorMap_[i], footMarkerDiameter_, footAlphaWhenLifted_));
    // Get force marker
    markerArray.markers.emplace_back(getForceMarker(feetForces[i], feetPositions[i], contactFlags[i], Color::green, forceScale_));
  }

  // Center of pressure
  // Get center of pressure marker
  markerArray.markers.emplace_back(getCenterOfPressureMarker(feetForces.begin(), feetForces.end(), feetPositions.begin(),
                                                             contactFlags.begin(), Color::green, copMarkerDiameter_));

  // Support polygon
  // Get support polygon marker
  markerArray.markers.emplace_back(
      getSupportPolygonMarker(feetPositions.begin(), feetPositions.end(), contactFlags.begin(), Color::black, supportPolygonLineWidth_));

  // Give markers an id and a frame
  // Assign header to markers
  assignHeader(markerArray.markers.begin(), markerArray.markers.end(), getHeaderMsg(frameId_, timeStamp));
  assignIncreasingId(markerArray.markers.begin(), markerArray.markers.end());

  // Publish cartesian markers (minus the CoM Pose)
  currentStatePublisher_.publish(markerArray);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*概述：LeggedRobotVisualizer::publishDesiredTrajectory 函数用于发布期望的轨迹信息，包括机器人的基础位置和各个接触点的位置。*/
void WheelLeggedRobotVisualizer::publishDesiredTrajectory(ros::Time timeStamp, const TargetTrajectories& targetTrajectories) {
  // 获取目标轨迹的状态轨迹和输入轨迹
  const auto& stateTrajectory = targetTrajectories.stateTrajectory;
  const auto& inputTrajectory = targetTrajectories.inputTrajectory;

  // Reserve com messages
  std::vector<geometry_msgs::Point> desiredBasePositionMsg;
  desiredBasePositionMsg.reserve(stateTrajectory.size());

  // Reserve feet messages
  feet_array_t<std::vector<geometry_msgs::Point>> desiredFeetPositionMsgs;
  for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
    desiredFeetPositionMsgs[i].reserve(stateTrajectory.size());
  }

  for (size_t j = 0; j < stateTrajectory.size(); j++) {
    const auto state = stateTrajectory.at(j);
    vector_t input(centroidalModelInfo_.inputDim);
    if (j < inputTrajectory.size()) {
      input = inputTrajectory.at(j);
    } else {
      input.setZero();
    }

    // Construct base pose msg
    const auto basePose = centroidal_model::getBasePose(state, centroidalModelInfo_);
    geometry_msgs::Pose pose;
    pose.position = getPointMsg(basePose.head<3>());

    // Fill message containers
    desiredBasePositionMsg.push_back(pose.position);

    // Fill feet msgs
    const auto& model = pinocchioInterface_.getModel();
    auto& data = pinocchioInterface_.getData();
    //  前向运动学计算 ，计算末端执行器的位置
    pinocchio::forwardKinematics(model, data, centroidal_model::getGeneralizedCoordinates(state, centroidalModelInfo_)); 
    pinocchio::updateFramePlacements(model, data); //  更新帧位置

    const auto feetPositions = endEffectorKinematicsPtr_->getPosition(state); //  获取末端执行器的位置
    for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
      geometry_msgs::Pose footPose;
      footPose.position = getPointMsg(feetPositions[i]); //  获取末端执行器的位置信息
      desiredFeetPositionMsgs[i].push_back(footPose.position); //  将末端执行器的位置信息添加到desiredFeetPositionMsgs中
    }
  }

  // Headers
  auto comLineMsg = getLineMsg(std::move(desiredBasePositionMsg), Color::green, trajectoryLineWidth_); //  获取comLineMsg
  comLineMsg.header = getHeaderMsg(frameId_, timeStamp); //  设置header
  comLineMsg.id = 0; //  设置id

  // Publish
  costDesiredBasePositionPublisher_.publish(comLineMsg); //  发布comLineMsg
  for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
    auto footLineMsg = getLineMsg(std::move(desiredFeetPositionMsgs[i]), feetColorMap_[i], trajectoryLineWidth_); //  获取footLineMsg
    footLineMsg.header = getHeaderMsg(frameId_, timeStamp); //  设置header
    footLineMsg.id = 0; //  设置id
    costDesiredFeetPositionPublishers_[i].publish(footLineMsg); //  发布footLineMsg
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*概述：LeggedRobotVisualizer::publishOptimizedStateTrajectory 函数用于发布优化后的状态轨迹。
       该函数接收时间戳、优化后的时间轨迹、优化后的状态轨迹以及模式调度，并将这些信息发布到 ROS 话题上。*/
void WheelLeggedRobotVisualizer::publishOptimizedStateTrajectory(ros::Time timeStamp, const scalar_array_t& mpcTimeTrajectory,
                                                            const vector_array_t& mpcStateTrajectory, const ModeSchedule& modeSchedule) {
  // Check if the mpcTimeTrajectory and mpcStateTrajectory are empty
  if (mpcTimeTrajectory.empty() || mpcStateTrajectory.empty()) { //  Check if the mpcTimeTrajectory and mpcStateTrajectory are empty
    return;  // Nothing to publish
  }

  // Reserve Feet msg
  feet_array_t<std::vector<geometry_msgs::Point>> feetMsgs;
  std::for_each(feetMsgs.begin(), feetMsgs.end(), [&](std::vector<geometry_msgs::Point>& v) { v.reserve(mpcStateTrajectory.size()); }); //  Reserve space for each foot in the feetMsgs

  // Reserve Com Msg
  std::vector<geometry_msgs::Point> mpcComPositionMsgs;
  mpcComPositionMsgs.reserve(mpcStateTrajectory.size());

  // Extract Com and Feet from state
  std::for_each(mpcStateTrajectory.begin(), mpcStateTrajectory.end(), [&](const vector_t& state) { //  Iterate through each state in the mpcStateTrajectory
    const auto basePose = centroidal_model::getBasePose(state, centroidalModelInfo_); //  获取质心位置和姿态

    // Fill com position and pose msgs
    geometry_msgs::Pose pose;
    pose.position = getPointMsg(basePose.head<3>()); //  获取basePose的前3个元素，并将其转换为PointMsg类型
    mpcComPositionMsgs.push_back(pose.position); //  将position添加到mpcComPositionMsgs中

    // Fill feet msgs
    const auto& model = pinocchioInterface_.getModel(); //  获取pinocchioInterface_的模型
    auto& data = pinocchioInterface_.getData(); //  获取模型
    pinocchio::forwardKinematics(model, data, centroidal_model::getGeneralizedCoordinates(state, centroidalModelInfo_)); //  获取数据
    pinocchio::updateFramePlacements(model, data); //  前向运动学计算

    const auto feetPositions = endEffectorKinematicsPtr_->getPosition(state); //  获取末端执行器的位置
    for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) { //  获取脚部位置
      const auto position = getPointMsg(feetPositions[i]); //  将位置转换为PointMsg类型
      feetMsgs[i].push_back(position); //  获取脚部位置消息
    }
  });

  // Convert feet msgs to Array message
  visualization_msgs::MarkerArray markerArray; //  创建MarkerArray消息
  markerArray.markers.reserve(centroidalModelInfo_.numThreeDofContacts + //  预留空间
                              2);  // 1 trajectory per foot + 1 for the future footholds + 1 for the com trajectory
  for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) { //  预留空间
    markerArray.markers.emplace_back(getLineMsg(std::move(feetMsgs[i]), feetColorMap_[i], trajectoryLineWidth_)); //  获取每只脚的轨迹消息
    markerArray.markers.back().ns = "EE Trajectories"; //  获取线消息
  }
  markerArray.markers.emplace_back(getLineMsg(std::move(mpcComPositionMsgs), Color::red, trajectoryLineWidth_)); //  Add com trajectory to the marker array
  markerArray.markers.back().ns = "CoM Trajectory"; //  获取线消息

  // Future footholds
  visualization_msgs::Marker sphereList; //  创建一个Marker类型的变量sphereList，用于存储球体列表
  sphereList.type = visualization_msgs::Marker::SPHERE_LIST; //  设置sphereList的类型为球体列表
  sphereList.scale.x = footMarkerDiameter_; //  设置标记类型为球体列表
  sphereList.scale.y = footMarkerDiameter_;
  sphereList.scale.z = footMarkerDiameter_;
  sphereList.ns = "Future footholds"; //  设置标记的尺寸
  sphereList.pose.orientation = getOrientationMsg({1., 0., 0., 0.}); //  设置标记的命名空间
  const auto& eventTimes = modeSchedule.eventTimes; //  设置标记的姿态
  const auto& subsystemSequence = modeSchedule.modeSequence; //  获取事件时间
  const auto tStart = mpcTimeTrajectory.front(); //  获取子系统序列
  const auto tEnd = mpcTimeTrajectory.back(); //  获取优化时间轨迹的起始时间
  for (size_t event = 0; event < eventTimes.size(); ++event) { //  获取优化时间轨迹的结束时间
    if (tStart < eventTimes[event] && eventTimes[event] < tEnd) {  // Only publish future footholds within the optimized horizon
      const auto preEventContactFlags = modeNumber2StanceLeg(subsystemSequence[event]); //  只发布优化时间范围内的未来足部支撑点
      const auto postEventContactFlags = modeNumber2StanceLeg(subsystemSequence[event + 1]); //  获取事件前的足部支撑点标志
      const auto postEventState = LinearInterpolation::interpolate(eventTimes[event], mpcTimeTrajectory, mpcStateTrajectory); //  获取事件后的足部支撑点标志

      const auto& model = pinocchioInterface_.getModel(); //  获取事件后的状态
      auto& data = pinocchioInterface_.getData(); //  获取模型
      pinocchio::forwardKinematics(model, data, centroidal_model::getGeneralizedCoordinates(postEventState, centroidalModelInfo_)); //  获取数据
      pinocchio::updateFramePlacements(model, data); //  前向运动学计算

      const auto feetPosition = endEffectorKinematicsPtr_->getPosition(postEventState); //  更新帧位置
      for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) { //  获取足部位置
        if (!preEventContactFlags[i] && postEventContactFlags[i]) {  // If a foot lands, a marker is added at that location.
          sphereList.points.emplace_back(getPointMsg(feetPosition[i]));
          sphereList.colors.push_back(getColor(feetColorMap_[i]));
        }
      }
    }
  }
  markerArray.markers.push_back(std::move(sphereList));

  // Add headers and Id
  assignHeader(markerArray.markers.begin(), markerArray.markers.end(), getHeaderMsg(frameId_, timeStamp)); //  给markerArray.markers中的每个marker添加header
  assignIncreasingId(markerArray.markers.begin(), markerArray.markers.end()); //  给markerArray.markers中的每个marker分配递增的id

  stateOptimizedPublisher_.publish(markerArray);
}

}  // namespace legged_robot
}  // namespace ocs2
