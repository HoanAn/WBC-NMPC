#include <string>

#include <hrp4_locomotion/JointCommand.hpp>

namespace labrob {

double
JointCommand::operator[](const std::string& key) const {
  return joint_command_.at(key);
}

double&
JointCommand::operator[](const std::string& key) {
  return joint_command_[key];
}

JointCommand::JointCommandMap::iterator
JointCommand::begin() {
  return joint_command_.begin();
}

JointCommand::JointCommandMap::iterator
JointCommand::end() {
  return joint_command_.end();
}

} // end namespace labrob