#ifndef LABROB_WALKING_STATE_HPP_
#define LABROB_WALKING_STATE_HPP_

#include <string>

namespace labrob {
/*! State of the robot regarding walking phases. */
enum class WalkingState {
  Init, /*!< The robot is in initial posture. */
  PostureRegulation, /*!< The robot is performing a posture regulation to go to Standing state. */
  Standing, /*!< The robot is standing, no footstep plan to execute. */
  Starting, /*!< A footstep plan is available, moving equilibrium from static to dynamic. */
  SingleSupport, /*!< Single support phase, keep ZMP within support polygon. */
  DoubleSupport, /*!< Double support phase, keep ZMP within support polygon. */
  Stopping, /*!< Similar to double support phase, but executing last element of footstep plan. */
  AbortStarting, /*!< Abort starting phase in order to go back to Standing without swing motion. */
  AbortWalking /*! Abort walking phase in order to go back to Standing without swing motion. */
};

std::string to_string(const WalkingState& walking_state);

} // end namespace labrob

#endif // LABROB_WALKING_STATE_HPP_