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

#include "wheel_legged_interface/constraint/FrictionConeConstraint.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>

namespace ocs2 {
namespace wheel_legged {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*概述：FrictionConeConstraint 构造函数用于初始化一个摩擦锥约束对象。

参数：

referenceManager：一个 SwitchedModelReferenceManager 类型的引用，用于管理参考模型。
config：一个 Config 类型的对象，包含配置信息。
contactPointIndex：一个 size_t 类型的整数，表示接触点的索引。
info：一个 CentroidalModelInfo 类型的对象，包含质心模型的信息。
返回值：无返回值，构造函数是用于初始化对象，不返回任何值。*/
FrictionConeConstraint::FrictionConeConstraint(const SwitchedModelReferenceManager& referenceManager, Config config,
                                               size_t contactPointIndex, CentroidalModelInfo info)
    : StateInputConstraint(ConstraintOrder::Quadratic),
      referenceManagerPtr_(&referenceManager),
      config_(std::move(config)),
      contactPointIndex_(contactPointIndex),
      info_(std::move(info)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*概述：FrictionConeConstraint 类的 setSurfaceNormalInWorld 方法用于设置约束在全局坐标系中的表面法线。
       如果该方法被调用，则会抛出一个运行时错误，表示该方法尚未实现。

参数：

surfaceNormalInWorld：一个 vector3_t 类型的参数，表示在全局坐标系中的表面法线向量。
返回值：

该方法没有返回值，因为它会抛出一个异常。
注意事项：

该方法目前没有实现，调用时会抛出 std::runtime_error 异常，提示 "[FrictionConeConstraint] setSurfaceNormalInWorld() is not implemented!"。*/
void FrictionConeConstraint::setSurfaceNormalInWorld(const vector3_t& surfaceNormalInWorld) {
  t_R_w.setIdentity();
  throw std::runtime_error("[FrictionConeConstraint] setSurfaceNormalInWorld() is not implemented!");
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*概述：该函数用于检查某个接触点在给定时间是否处于活动状态。

参数：

time：时间点，类型为 scalar_t，表示当前的时间。
返回值：

返回一个布尔值，表示接触点在给定时间是否处于活动状态。如果接触点处于活动状态，则返回 true，否则返回 false。*/
bool FrictionConeConstraint::isActive(scalar_t time) const {
  //  获取referenceManagerPtr_中contactPointIndex_对应的contactFlags
  return referenceManagerPtr_->getContactFlags(time)[contactPointIndex_]; 
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*概述：FrictionConeConstraint::getValue 函数用于计算给定时间、状态、输入和预计算数据的情况下，摩擦锥约束的值。

参数：

time (scalar_t): 当前的时间点。
state (vector_t): 系统的状态向量。
input (vector_t): 系统的输入向量。
preComp (PreComputation): 预计算数据，包含一些中间计算结果。
返回值：

返回一个 vector_t 类型的值，表示摩擦锥约束的计算结果。
详细说明：

函数首先调用 centroidal_model::getContactForces 函数，根据输入和接触点索引，获取在世界坐标系下的接触力。
然后，将接触力从世界坐标系转换到局部坐标系，转换矩阵为 t_R_w。
最后，调用 coneConstraint 函数，计算摩擦锥约束的值，并返回结果。*/
vector_t FrictionConeConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                          const PreComputation& preComp) const {
  //  获取在世界坐标系下的接触力
  const auto forcesInWorldFrame = centroidal_model::getContactForces(input, contactPointIndex_, info_); 
  const vector3_t localForce = t_R_w * forcesInWorldFrame; //  将接触力转换到本地坐标系
  return coneConstraint(localForce); //  返回摩擦锥约束的值
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*概述：该函数用于获取摩擦锥约束的线性近似。
       它根据给定的时间和状态，计算在给定输入和预计算数据的情况下，摩擦力在全局坐标系中的值，并返回一个线性近似。

参数：

time：时间，类型为 scalar_t。
state：系统状态，类型为 vector_t。
input：输入，类型为 vector_t。
preComp：预计算数据，类型为 PreComputation。
返回值：

返回一个 VectorFunctionLinearApproximation 类型的对象，其中包含摩擦力在全局坐标系中的线性近似。
详细说明：

forcesInWorldFrame：计算输入在全局坐标系中的接触力。
linearApproximation.dfdx：初始化为全零矩阵，表示状态对摩擦力的导数。
linearApproximation.dfdu：计算输入对摩擦力的导数，并赋值给 linearApproximation.dfdu。
最后，返回 linearApproximation 对象，包含摩擦力的线性近似。*/
VectorFunctionLinearApproximation FrictionConeConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                 const vector_t& input,
                                                                                 const PreComputation& preComp) const {
  //  获取在世界坐标系下的接触力
  const vector3_t forcesInWorldFrame = centroidal_model::getContactForces(input, contactPointIndex_, info_); 
  const vector3_t localForce = t_R_w * forcesInWorldFrame; //  将接触力转换到本地坐标系

  //  计算本地接触力的导数
  const auto localForceDerivatives = computeLocalForceDerivatives(forcesInWorldFrame); 
  //  计算摩擦锥的本地导数
  const auto coneLocalDerivatives = computeConeLocalDerivatives(localForce); 
   //  计算摩擦锥约束的导数
  const auto coneDerivatives = computeConeConstraintDerivatives(coneLocalDerivatives, localForceDerivatives);

  VectorFunctionLinearApproximation linearApproximation; //  创建线性近似
  linearApproximation.f = coneConstraint(localForce); //  设置线性近似的函数值
  linearApproximation.dfdx = matrix_t::Zero(1, state.size()); //  设置线性近似的导数
  linearApproximation.dfdu = frictionConeInputDerivative(input.size(), coneDerivatives);//  设置线性近似的输入导数
  return linearApproximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*概述：获取摩擦锥约束的二次近似。

参数：

time：时间，类型为 scalar_t。
state：系统状态，类型为 vector_t。
input：系统输入，类型为 vector_t。
preComp：预计算数据，类型为 PreComputation。
返回值：

返回一个 VectorFunctionQuadraticApproximation 类型的对象，表示摩擦锥约束的二次近似。*/
VectorFunctionQuadraticApproximation FrictionConeConstraint::getQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                                       const vector_t& input,
                                                                                       const PreComputation& preComp) const {
  //  在世界坐标系中获取接触力
  const vector3_t forcesInWorldFrame = centroidal_model::getContactForces(input, contactPointIndex_, info_); 
  const vector3_t localForce = t_R_w * forcesInWorldFrame; //  将接触力转换到本地坐标系

  const auto localForceDerivatives = computeLocalForceDerivatives(forcesInWorldFrame); //  计算本地接触力的导数
  const auto coneLocalDerivatives = computeConeLocalDerivatives(localForce); //  计算摩擦锥的本地导数
  //  计算摩擦锥约束的导数
  const auto coneDerivatives = computeConeConstraintDerivatives(coneLocalDerivatives, localForceDerivatives); 

  VectorFunctionQuadraticApproximation quadraticApproximation; //  创建二次近似
  quadraticApproximation.f = coneConstraint(localForce); //  设置二次近似的函数值
  quadraticApproximation.dfdx = matrix_t::Zero(1, state.size()); //  设置二次近似的函数对状态变量的导数
  //  设置二次近似的函数对输入变量的导数
  quadraticApproximation.dfdu = frictionConeInputDerivative(input.size(), coneDerivatives); 
   //  设置二次近似的函数对状态变量的二阶导数
  quadraticApproximation.dfdxx.emplace_back(frictionConeSecondDerivativeState(state.size(), coneDerivatives));
   //  设置二次近似的函数对输入变量的二阶导数
  quadraticApproximation.dfduu.emplace_back(frictionConeSecondDerivativeInput(input.size(), coneDerivatives));
  //  设置二次近似的函数对状态变量和输入变量的二阶导数
  quadraticApproximation.dfdux.emplace_back(matrix_t::Zero(input.size(), state.size())); 
  return quadraticApproximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*概述：计算摩擦锥约束的局部力导数。

参数：

forcesInWorldFrame：在世界坐标系中的力向量。
返回值：

LocalForceDerivatives：包含局部力导数的结构体，其中 dF_du 成员被设置为 t_R_w。*/
FrictionConeConstraint::LocalForceDerivatives FrictionConeConstraint::computeLocalForceDerivatives(
    const vector3_t& forcesInWorldFrame) const {
  LocalForceDerivatives localForceDerivatives{}; //  创建局部力导数对象
  localForceDerivatives.dF_du = t_R_w; //  计算局部力导数
  return localForceDerivatives; //  返回局部力导数
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*概述：计算摩擦锥约束的局部导数。

参数：

localForces：一个包含局部力的向量，类型为 vector3_t。
返回值：

返回一个 ConeLocalDerivatives 类型的对象，该对象包含了摩擦锥约束的局部导数。
具体来说，该函数计算并返回摩擦锥约束的局部导数矩阵。局部导数矩阵用于描述摩擦锥约束对局部力的敏感度。
函数首先计算局部力的平方和，然后计算摩擦锥的导数矩阵，并返回该矩阵。*/
FrictionConeConstraint::ConeLocalDerivatives FrictionConeConstraint::computeConeLocalDerivatives(const vector3_t& localForces) const {
  const auto F_x_square = localForces.x() * localForces.x(); //  计算局部力在x轴上的平方
  const auto F_y_square = localForces.y() * localForces.y(); //  计算局部力在y轴上的平方
  const auto F_tangent_square = F_x_square + F_y_square + config_.regularization; //  计算切线方向的力平方
  const auto F_tangent_norm = sqrt(F_tangent_square); //  计算切线方向的力模
   //  计算切线方向的力平方的(3/2)次方
  const auto F_tangent_square_pow32 = F_tangent_norm * F_tangent_square;  // = F_tangent_square ^ (3/2)

  ConeLocalDerivatives coneDerivatives{}; //  初始化摩擦锥局部导数
  coneDerivatives.dCone_dF(0) = -localForces.x() / F_tangent_norm; //  计算摩擦锥局部导数在x轴上的导数
  coneDerivatives.dCone_dF(1) = -localForces.y() / F_tangent_norm; //  计算摩擦锥局部导数在y轴上的导数
  coneDerivatives.dCone_dF(2) = config_.frictionCoefficient; //  计算摩擦锥局部导数在摩擦系数上的导数

  //  计算摩擦锥局部导数在x轴上的二阶导数
  coneDerivatives.d2Cone_dF2(0, 0) = -(F_y_square + config_.regularization) / F_tangent_square_pow32; 
  //  计算摩擦锥局部导数在x轴和y轴上的二阶导数
  coneDerivatives.d2Cone_dF2(0, 1) = localForces.x() * localForces.y() / F_tangent_square_pow32;
  coneDerivatives.d2Cone_dF2(0, 2) = 0.0; //  计算摩擦锥局部导数在x轴和摩擦系数上的二阶导数
  coneDerivatives.d2Cone_dF2(1, 0) = coneDerivatives.d2Cone_dF2(0, 1); //  计算摩擦锥局部导数在y轴和x轴上的二阶导数
  //  计算摩擦锥局部导数在y轴上的二阶导数
  coneDerivatives.d2Cone_dF2(1, 1) = -(F_x_square + config_.regularization) / F_tangent_square_pow32; 
  coneDerivatives.d2Cone_dF2(1, 2) = 0.0; //  计算摩擦锥局部导数在y轴和摩擦系数上的二阶导数
  coneDerivatives.d2Cone_dF2(2, 0) = 0.0; //  计算摩擦锥局部导数在摩擦系数上的二阶导数
  coneDerivatives.d2Cone_dF2(2, 1) = 0.0;
  coneDerivatives.d2Cone_dF2(2, 2) = 0.0;

  return coneDerivatives;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*概述：该函数计算并返回一个摩擦锥约束值。摩擦锥约束用于确保在物理模拟中，物体的力不会超过摩擦锥的限制。

参数：

localForces：一个三维向量，表示局部坐标系下的力。该向量包含三个分量：x、y 和 z。
返回值：

返回一个一维向量，包含计算得到的摩擦锥约束值。
具体来说，函数首先计算局部力在切平面上的平方和，并加上一个正则化项。
然后计算切平面上力的模长。
接着，根据摩擦系数和局部力在垂直方向上的分量（加上一个抓取力），计算摩擦锥约束值。
最后，将这个约束值封装在一个一维向量中并返回。*/
vector_t FrictionConeConstraint::coneConstraint(const vector3_t& localForces) const {
  //  计算切向力的平方
  const auto F_tangent_square = localForces.x() * localForces.x() + localForces.y() * localForces.y() + config_.regularization; 
  //  计算切向力的模长
  const auto F_tangent_norm = sqrt(F_tangent_square); 
  //  计算摩擦锥约束值
  const scalar_t coneConstraint = config_.frictionCoefficient * (localForces.z() + config_.gripperForce) - F_tangent_norm; 
  return (vector_t(1) << coneConstraint).finished();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*概述：计算摩擦锥约束的导数。

参数：

coneLocalDerivatives：摩擦锥的局部导数，类型为 ConeLocalDerivatives。
localForceDerivatives：局部力导数，类型为 LocalForceDerivatives。
返回值：

返回一个 ConeDerivatives 类型的对象，包含摩擦锥约束的一阶和二阶导数。
具体来说，该函数计算摩擦锥约束的一阶和二阶导数，并返回一个包含这些导数的 ConeDerivatives 对象。
一阶导数通过局部导数和局部力导数的乘积计算得到，二阶导数通过局部力导数的平方和局部导数的二阶导数的乘积计算得到。
*/
FrictionConeConstraint::ConeDerivatives FrictionConeConstraint::computeConeConstraintDerivatives(
    const ConeLocalDerivatives& coneLocalDerivatives, const LocalForceDerivatives& localForceDerivatives) const {
  ConeDerivatives coneDerivatives;
  // First order derivatives
  //  计算锥约束的一阶导数
  coneDerivatives.dCone_du.noalias() = coneLocalDerivatives.dCone_dF.transpose() * localForceDerivatives.dF_du; 

  // Second order derivatives
   //  计算锥约束的二阶导数
  coneDerivatives.d2Cone_du2.noalias() =
      localForceDerivatives.dF_du.transpose() * coneLocalDerivatives.d2Cone_dF2 * localForceDerivatives.dF_du;

  return coneDerivatives;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*概述：计算摩擦锥约束的输入导数。

参数：

inputDim：输入维度，表示输入向量的长度。
coneDerivatives：摩擦锥的导数，类型为 ConeDerivatives，包含摩擦锥关于输入向量的导数。
返回值：

返回一个 matrix_t 类型的矩阵，表示摩擦锥约束的输入导数。矩阵的形状为 (1, inputDim)，其中第一行存储了导数信息。*/
matrix_t FrictionConeConstraint::frictionConeInputDerivative(size_t inputDim, const ConeDerivatives& coneDerivatives) const {
  matrix_t dhdu = matrix_t::Zero(1, inputDim); //  初始化导数为零矩阵
  dhdu.block<1, 3>(0, 3 * contactPointIndex_) = coneDerivatives.dCone_du; //  将摩擦锥的导数赋值给导数矩阵的对应位置
  return dhdu;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*概述：该函数计算并返回一个矩阵，该矩阵表示摩擦锥约束的二次导数输入。该矩阵用于描述在给定输入维度和摩擦锥导数的情况下，约束的二次导数。

参数：

inputDim：输入维度，表示输入向量的长度。
coneDerivatives：一个包含摩擦锥导数的对象，具体包含二次导数 d2Cone_du2。
返回值：

返回一个 matrix_t 类型的矩阵，该矩阵表示摩擦锥约束的二次导数输入。
具体来说，该函数首先创建一个全零矩阵 ddhdudu，其维度为 inputDim x inputDim。
然后，将 coneDerivatives.d2Cone_du2 的值赋给矩阵 ddhdudu 中从 3 * contactPointIndex_ 开始的 3 x 3 块。
接着，从矩阵 ddhdudu 的对角线元素中减去 config_.hessianDiagonalShift 的值。
最后，返回修改后的矩阵 ddhdudu。*/
matrix_t FrictionConeConstraint::frictionConeSecondDerivativeInput(size_t inputDim, const ConeDerivatives& coneDerivatives) const {
  matrix_t ddhdudu = matrix_t::Zero(inputDim, inputDim); //  初始化二次导数输入矩阵为0矩阵
  //  将摩擦锥的二次导数输入矩阵赋值给二次导数输入矩阵的对应位置
  ddhdudu.block<3, 3>(3 * contactPointIndex_, 3 * contactPointIndex_) = coneDerivatives.d2Cone_du2; 
  //  将二次导数输入矩阵的对角线元素减去配置中的海森矩阵对角线偏移量
  ddhdudu.diagonal().array() -= config_.hessianDiagonalShift; 
  return ddhdudu;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*概述：计算摩擦锥约束的二次导数状态矩阵。

参数：

stateDim：状态维度，表示系统状态向量的维度。
coneDerivatives：摩擦锥导数，包含摩擦锥相关的导数信息。
返回值：

返回一个 matrix_t 类型的矩阵，表示摩擦锥约束的二次导数状态矩阵。
该矩阵是一个 stateDim x stateDim 的零矩阵，对角线元素减去配置中的海森矩阵对角线偏移量。
*/
matrix_t FrictionConeConstraint::frictionConeSecondDerivativeState(size_t stateDim, const ConeDerivatives& coneDerivatives) const {
  matrix_t ddhdxdx = matrix_t::Zero(stateDim, stateDim); //  初始化二次导数矩阵为0矩阵
  ddhdxdx.diagonal().array() -= config_.hessianDiagonalShift; //  将对角线元素减去配置中的海森矩阵对角线偏移量
  return ddhdxdx;
}

}  // namespace legged_robot
}  // namespace ocs2
