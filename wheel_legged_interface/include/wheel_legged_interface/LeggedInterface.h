#pragma clang diagnostic push
#pragma ide diagnostic ignored "misc-non-private-member-variables-in-classes"
//
// Created by qiayuan on 2022/7/16.
//

#pragma once

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/Types.h>
#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_ipm/IpmSettings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>
#include <ocs2_self_collision/PinocchioGeometryInterface.h>
#include <ocs2_sqp/SqpSettings.h>

#include "wheel_legged_interface/common/ModelSettings.h"
#include "wheel_legged_interface/initialization/LeggedRobotInitializer.h"
#include "wheel_legged_interface/reference_manager/SwitchedModelReferenceManager.h"

namespace ocs2 {
namespace wheel_legged {

class WheelLeggedInterface : public RobotInterface {
 public:
  WheelLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                  bool useHardFrictionConeConstraint = false);

  ~WheelLeggedInterface() override = default;

  /*virtual void setupOptimalControlProblem(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                          bool verbose);*/

  const OptimalControlProblem& getOptimalControlProblem() const override { return *problemPtr_; }

  const ModelSettings& modelSettings() const { return modelSettings_; }
  const ddp::Settings& ddpSettings() const { return ddpSettings_; }
  const mpc::Settings& mpcSettings() const { return mpcSettings_; }
  const rollout::Settings& rolloutSettings() const { return rolloutSettings_; }
  const sqp::Settings& sqpSettings() { return sqpSettings_; }
  const ipm::Settings& ipmSettings() { return ipmSettings_; }

  const vector_t& getInitialState() const { return initialState_; }
  const RolloutBase& getRollout() const { return *rolloutPtr_; }
  PinocchioInterface& getPinocchioInterface() { return *pinocchioInterfacePtr_; }
  const CentroidalModelInfo& getCentroidalModelInfo() const { return centroidalModelInfo_; }
  PinocchioGeometryInterface& getGeometryInterface() { return *geometryInterfacePtr_; }
  std::shared_ptr<SwitchedModelReferenceManager> getSwitchedModelReferenceManagerPtr() const { return referenceManagerPtr_; }

  const Initializer& getInitializer() const override { return *initializerPtr_; }
  std::shared_ptr<ReferenceManagerInterface> getReferenceManagerPtr() const override { return referenceManagerPtr_; }

  void setupOptimalControlProblem(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile, bool verbose);
 protected:
  /*virtual void setupModel(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile, bool verbose);
  virtual void setupReferenceManager(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                     bool verbose);
  virtual void setupPreComputation(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                   bool verbose);*/

  std::shared_ptr<GaitSchedule> loadGaitSchedule(const std::string& file, bool verbose) const;

  std::unique_ptr<StateInputCost> getBaseTrackingCost(const std::string& taskFile, const CentroidalModelInfo& info, bool verbose);
  matrix_t initializeInputCostWeight(const std::string& taskFile, const CentroidalModelInfo& info);

  static std::pair<scalar_t, RelaxedBarrierPenalty::Config> loadFrictionConeSettings(const std::string& taskFile, bool verbose);
  std::unique_ptr<StateInputConstraint> getFrictionConeConstraint(size_t contactPointIndex, scalar_t frictionCoefficient);
  std::unique_ptr<StateInputCost> getFrictionConeSoftConstraint(size_t contactPointIndex, scalar_t frictionCoefficient,
                                                                const RelaxedBarrierPenalty::Config& barrierPenaltyConfig);
  std::unique_ptr<StateInputConstraint> getZeroForceConstraint(size_t contactPointIndex);
  /*std::unique_ptr<EndEffectorKinematics<scalar_t>> getEeKinematicsPtr(const std::vector<std::string>& footNames,
                                                                      const std::string& modelName);*/
  std::unique_ptr<StateInputConstraint> getZeroVelocityConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics,
                                                                  size_t contactPointIndex, bool useAnalyticalGradients);
  std::unique_ptr<StateInputConstraint> getNormalVelocityConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics,
                                                                    size_t contactPointIndex, bool useAnalyticalGradients);
  std::unique_ptr<StateCost> getSelfCollisionConstraint(const PinocchioInterface& pinocchioInterface, const std::string& taskFile,
                                                        const std::string& prefix, bool verbose);
  std::unique_ptr<StateInputConstraint> getWheelRollConstraint(size_t contactPointIndex);

  ModelSettings modelSettings_;
  mpc::Settings mpcSettings_;
  ddp::Settings ddpSettings_;
  sqp::Settings sqpSettings_;
  ipm::Settings ipmSettings_;
  const bool useHardFrictionConeConstraint_;

  std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr_;
  CentroidalModelInfo centroidalModelInfo_;
  std::unique_ptr<PinocchioGeometryInterface> geometryInterfacePtr_;

  std::unique_ptr<OptimalControlProblem> problemPtr_;
  std::shared_ptr<SwitchedModelReferenceManager> referenceManagerPtr_;

  rollout::Settings rolloutSettings_;
  std::unique_ptr<RolloutBase> rolloutPtr_;
  std::unique_ptr<Initializer> initializerPtr_;

  vector_t initialState_;
};

}  // namespace legged
}
#pragma clang diagnostic pop
