#ifndef LABROB_JOINT_COMMAND_H_
#define LABROB_JOINT_COMMAND_H_

// STL
#include <string>
#include <unordered_map>

namespace labrob {

class JointCommand {
  using JointCommandMap = std::unordered_map<std::string, double>;
 public:
  double operator[](const std::string& key) const;

  double& operator[](const std::string& key);

  JointCommandMap::iterator begin();

  JointCommandMap::iterator end();

 protected:
  JointCommandMap joint_command_;
}; // end class JointCommand

} // end namespace labrob

#endif // LABROB_JOINT_COMMAND_H_