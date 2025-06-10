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

#include "wheel_legged_interface/constraint/EndEffectorLinearConstraint.h"

namespace ocs2 {
namespace wheel_legged {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*概述：EndEffectorLinearConstraint 类的构造函数，用于创建一个线性约束，该约束基于末端执行器的运动学模型。

参数：

endEffectorKinematics：一个 EndEffectorKinematics<scalar_t> 类型的对象，表示末端执行器的运动学模型。
numConstraints：一个 size_t 类型的整数，表示约束的数量。
config：一个 Config 类型的对象，表示配置参数。
返回值：无返回值。构造函数是用于初始化 EndEffectorLinearConstraint 对象的，不返回任何值。

注意事项：

构造函数首先调用基类 StateInputConstraint 的构造函数，并传入 ConstraintOrder::Linear 作为参数。
然后，它克隆 endEffectorKinematics 对象，并将其指针存储在 endEffectorKinematicsPtr_ 成员变量中。
接着，它将 numConstraints 存储在 numConstraints_ 成员变量中。
最后，它将 config 对象移动到 config_ 成员变量中。
如果 endEffectorKinematics 对象的 getIds() 方法返回的 ID 数量不是 1，则抛出一个运行时错误，指出该类只接受单个末端执行器。*/
EndEffectorLinearConstraint::EndEffectorLinearConstraint(const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                                         size_t numConstraints, Config config)
    : StateInputConstraint(ConstraintOrder::Linear),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      numConstraints_(numConstraints),
      config_(std::move(config)) {
  if (endEffectorKinematicsPtr_->getIds().size() != 1) {
    throw std::runtime_error("[EndEffectorLinearConstraint] this class only accepts a single end-effector!");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*概述：EndEffectorLinearConstraint类的拷贝构造函数，用于创建一个新的EndEffectorLinearConstraint对象，
       该对象是传入对象rhs的副本。

参数：

rhs：一个EndEffectorLinearConstraint类型的对象，作为拷贝构造函数的参数，用于初始化新对象。
返回值：

无返回值，因为这是一个构造函数，它的返回值是创建的新对象。*/
EndEffectorLinearConstraint::EndEffectorLinearConstraint(const EndEffectorLinearConstraint& rhs)
    : StateInputConstraint(rhs),
      endEffectorKinematicsPtr_(rhs.endEffectorKinematicsPtr_->clone()),
      numConstraints_(rhs.numConstraints_),
      config_(rhs.config_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*概述：EndEffectorLinearConstraint::configure 函数用于配置末端执行器的线性约束。
       它接受一个配置对象，并验证该对象中的矩阵和向量是否符合预期的大小和格式。

参数：

Config&& config：一个右值引用的配置对象，包含末端执行器的线性约束配置信息。
返回值：

无返回值。该函数通过修改成员变量 config_ 来配置对象。
详细说明：

函数首先断言 config.b 的行数等于 numConstraints_，确保约束的数量正确。
断言 config.Ax 或 config.Av 至少有一个不为空，确保至少有一个约束条件被提供。
如果 config.Ax 不为空，则断言其行数等于 numConstraints_，列数等于3，确保 Ax 矩阵的格式正确。
如果 config.Av 不为空，则断言其行数等于 numConstraints_，列数等于3，确保 Av 矩阵的格式正确。
最后，通过 std::move 将 config 移动到成员变量 config_ 中，完成配置。
这些断言确保了传入的配置对象符合预期的格式和大小，从而保证了配置的正确性和安全性。*/
void EndEffectorLinearConstraint::configure(Config&& config) {
  assert(config.b.rows() == numConstraints_);
  assert(config.Ax.size() > 0 || config.Av.size() > 0);
  assert((config.Ax.size() > 0 && config.Ax.rows() == numConstraints_) || config.Ax.size() == 0);
  assert((config.Ax.size() > 0 && config.Ax.cols() == 3) || config.Ax.size() == 0);
  assert((config.Av.size() > 0 && config.Av.rows() == numConstraints_) || config.Av.size() == 0);
  assert((config.Av.size() > 0 && config.Av.cols() == 3) || config.Av.size() == 0);
  config_ = std::move(config);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*概述：该函数用于计算末端执行器在给定时间、状态、输入和预计算数据下的线性约束值。

参数：

time：时间，类型为 scalar_t，表示当前的时间点。
state：状态向量，类型为 vector_t，表示系统的当前状态。
input：输入向量，类型为 vector_t，表示系统的当前输入。
preComp：预计算数据，类型为 PreComputation，包含了一些预先计算好的数据，用于加速计算。
返回值：

返回一个 vector_t 类型的向量，表示末端执行器的线性约束值。*/
vector_t EndEffectorLinearConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                               const PreComputation& preComp) const {
  vector_t f = config_.b;
  if (config_.Ax.size() > 0) {
    f.noalias() += config_.Ax * endEffectorKinematicsPtr_->getPosition(state).front();
  }
  if (config_.Av.size() > 0) {
    f.noalias() += config_.Av * endEffectorKinematicsPtr_->getVelocity(state, input).front();
  }
  return f;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*概述：该函数用于获取末端执行器线性约束的线性近似。它根据给定的时间和状态，计算末端执行器的位置和速度的线性近似，并返回这些近似值。

参数：

time：时间，类型为 scalar_t。
state：系统状态，类型为 vector_t。
input：系统输入，类型为 vector_t。
preComp：预计算数据，类型为 PreComputation。
返回值：

返回一个 VectorFunctionLinearApproximation 类型的对象，包含末端执行器线性约束的线性近似值。*/
VectorFunctionLinearApproximation EndEffectorLinearConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                      const vector_t& input,
                                                                                      const PreComputation& preComp) const {
  VectorFunctionLinearApproximation linearApproximation = //  创建一个线性近似对象，大小为约束数量、状态大小和输入大小
      VectorFunctionLinearApproximation::Zero(getNumConstraints(time), state.size(), input.size());

  linearApproximation.f = config_.b; //  设置线性近似对象的f值为config_中的b值

  if (config_.Ax.size() > 0) { //  如果config_中的Ax大小大于0，则获取末端执行器的位置线性近似
    const auto positionApprox = endEffectorKinematicsPtr_->getPositionLinearApproximation(state).front();
    linearApproximation.f.noalias() += config_.Ax * positionApprox.f; //  将位置线性近似的f值乘以Ax，并加到线性近似对象的f值上
    linearApproximation.dfdx.noalias() += config_.Ax * positionApprox.dfdx; //  将位置线性近似的dfdx值乘以Ax，并加到线性近似对象的dfdx值上
  }

  if (config_.Av.size() > 0) { //  如果config_中的Av大小大于0，则获取末端执行器的速度线性近似
    const auto velocityApprox = endEffectorKinematicsPtr_->getVelocityLinearApproximation(state, input).front();
    linearApproximation.f.noalias() += config_.Av * velocityApprox.f; //  将速度线性近似的f值乘以Av，并加到线性近似对象的f值上
    linearApproximation.dfdx.noalias() += config_.Av * velocityApprox.dfdx; //  将速度线性近似的dfdx值乘以Av，并加到线性近似对象的dfdx值上
    linearApproximation.dfdu.noalias() += config_.Av * velocityApprox.dfdu; //  将速度线性近似的dfdu值乘以Av，并加到线性近似对象的dfdu值上
  }

  return linearApproximation;
}

}  // namespace legged_robot
}  // namespace ocs2
