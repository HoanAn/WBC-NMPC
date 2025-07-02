#ifndef LABROB_WALKING_DATA_HPP_
#define LABROB_WALKING_DATA_HPP_

#include <deque>

#include <hrp4_locomotion/FootstepPlanElement.hpp>
#include <hrp4_locomotion/WalkingState.hpp>

namespace labrob {

class WalkingData {
 public:
  std::deque<labrob::FootstepPlanElement> footstep_plan;
  int64_t t0; // Starting time of footstep_plan.front()

  /**!
   * Returns walking state of first element of the footstep plan, which
   * corresponds to current walking state.
   */
  const labrob::WalkingState& getWalkingState() const;

  /**!
   * Update footstep plan with current stance to make it consistent with
   * sensor data.
  */
  void updateFootstepPlanWithCurrentStance(
      const labrob::SE3& leftFootConfiguration,
      const labrob::SE3& rightFootConfiguration
  );

  /**!
   * Update walking state and footstep plan with current time.
  */
  void updateWalkingState(int64_t t);

}; // end class WalkingData

} // end namespace labrob

#endif // LABROB_WALKING_DATA_HPP_