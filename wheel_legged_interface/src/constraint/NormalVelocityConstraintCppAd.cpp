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

#include "wheel_legged_interface/constraint/NormalVelocityConstraintCppAd.h"
#include "wheel_legged_interface/LeggedRobotPreComputation.h"

namespace ocs2 {
namespace wheel_legged {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
NormalVelocityConstraintCppAd::NormalVelocityConstraintCppAd(const SwitchedModelReferenceManager& referenceManager,
                                                             const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                                             size_t contactPointIndex)
    : StateInputConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      eeLinearConstraintPtr_(new EndEffectorLinearConstraint(endEffectorKinematics, 1)),
      contactPointIndex_(contactPointIndex) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
NormalVelocityConstraintCppAd::NormalVelocityConstraintCppAd(const NormalVelocityConstraintCppAd& rhs)
    : StateInputConstraint(rhs),
      referenceManagerPtr_(rhs.referenceManagerPtr_),
      eeLinearConstraintPtr_(rhs.eeLinearConstraintPtr_->clone()),
      contactPointIndex_(rhs.contactPointIndex_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool NormalVelocityConstraintCppAd::isActive(scalar_t time) const {
  return !referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t NormalVelocityConstraintCppAd::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                                 const PreComputation& preComp) const {
  //  将preComp转换为LeggedRobotPreComputation类型
  const auto& preCompLegged = cast<LeggedRobotPreComputation>(preComp); 
  //  配置末端执行器的线性约束
  eeLinearConstraintPtr_->configure(preCompLegged.getEeNormalVelocityConstraintConfigs()[contactPointIndex_]); 

  return eeLinearConstraintPtr_->getValue(time, state, input, preComp); //  返回末端执行器的线性约束的值
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation NormalVelocityConstraintCppAd::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                        const vector_t& input,
                                                                                        const PreComputation& preComp) const {
  //  将preComp转换为LeggedRobotPreComputation类型
  const auto& preCompLegged = cast<LeggedRobotPreComputation>(preComp); 
  //  配置末端执行器线性约束
  eeLinearConstraintPtr_->configure(preCompLegged.getEeNormalVelocityConstraintConfigs()[contactPointIndex_]);
  //  返回末端执行器线性约束的线性近似
  return eeLinearConstraintPtr_->getLinearApproximation(time, state, input, preComp); 
}

}  // namespace legged_robot
}  // namespace ocs2
