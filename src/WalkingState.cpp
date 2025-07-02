#include <hrp4_locomotion/WalkingState.hpp>

namespace labrob {
std::string to_string(const WalkingState& walking_state) {
  std::string walking_state_str;
  if (walking_state == WalkingState::Init) {
    walking_state_str = "Init";
  } else if (walking_state == WalkingState::PostureRegulation) {
    walking_state_str = "PostureRegulation";
  } else if (walking_state == WalkingState::Standing) {
    walking_state_str = "Standing";
  } else if (walking_state == WalkingState::Starting) {
    walking_state_str = "Starting";
  } else if (walking_state == WalkingState::SingleSupport) {
    walking_state_str = "SingleSupport";
  } else if (walking_state == WalkingState::DoubleSupport) {
    walking_state_str = "DoubleSupport";
  } else if (walking_state == WalkingState::Stopping) {
    walking_state_str = "Stopping";
  } else if (walking_state == WalkingState::AbortStarting) {
    walking_state_str = "AbortStarting";
  } else if (walking_state == WalkingState::AbortWalking) {
    walking_state_str = "AbortWalking";
  } else {
    // NOTE: execution should never get here.
    walking_state_str = "Unknown";
  }
  return walking_state_str;
}
} // end namespace labrob