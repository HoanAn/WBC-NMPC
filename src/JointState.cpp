#include <string>

#include <hrp4_locomotion/JointState.hpp>

namespace labrob {

labrob::JointData
JointState::operator[](const std::string& key) const {
  return joint_state_.at(key);
}

labrob::JointData&
JointState::operator[](const std::string& key) {
  return joint_state_[key];
}

JointState::JointStateMap::iterator
JointState::begin() {
  return joint_state_.begin();
}

JointState::JointStateMap::iterator
JointState::end() {
  return joint_state_.end();
}

} // end namespace labrob