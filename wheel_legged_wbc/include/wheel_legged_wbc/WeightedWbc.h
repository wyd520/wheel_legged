//
// Created by qiayuan on 22-12-23.
//

#include "wheel_legged_wbc/WbcBase.h"

namespace ocs2{
namespace wheel_legged {

class WeightedWbc : public WbcBase {
 public:
  using WbcBase::WbcBase;

  vector_t update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                  scalar_t period) override;

  void loadTasksSetting(const std::string& taskFile, bool verbose) override;

 protected:
  virtual Task formulateConstraints();
  virtual Task formulateWeightedTasks(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period);
  // virtual Task formulateHindFeetLateralVelocityTask();

 private:
  scalar_t weightSwingLeg_, weightBaseAccel_, weightContactForce_;
};
}
}  // namespace legged