#pragma once

#include <memory>

#include <ocs2_core/constraint/StateInputConstraint.h>

#include <ocs2_centroidal_model/CentroidalModelInfo.h>

#include "wheel_legged_interface/reference_manager/SwitchedModelReferenceManager.h"

namespace ocs2 {
namespace wheel_legged {

class WheelRollConstraint final : public StateInputConstraint {
 public:
    WheelRollConstraint(const SwitchedModelReferenceManager& referenceManager, size_t contactPointIndex,CentroidalModelInfo info);

    ~WheelRollConstraint() override = default;
    WheelRollConstraint* clone() const override
    {
       return new WheelRollConstraint(*this);
    }
    bool isActive(scalar_t time) const override;

    size_t getNumConstraints(scalar_t time) const override { return 1; }
    // 计算约束值（末端 y 方向速度）
    vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input,
                      const PreComputation& preComp) const override;
    VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                           const PreComputation& preComp) const override;
 private:
    WheelRollConstraint(const WheelRollConstraint& rhs);

    const SwitchedModelReferenceManager* referenceManagerPtr_;
    const size_t contactPointIndex_;
    const CentroidalModelInfo info_;
};

}
}