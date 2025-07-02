#ifndef LABROB_JOINT_STATE_H_
#define LABROB_JOINT_STATE_H_

// STL
#include <string>
#include <unordered_map>

namespace labrob {

struct JointData {
  double pos = 0.0;
  double vel = 0.0;
  double acc = 0.0;
  double eff = 0.0;
}; // end struct JointData

class JointState {
  using JointStateMap = std::unordered_map<std::string, labrob::JointData>;
 public:
  labrob::JointData operator[](const std::string& key) const;

  labrob::JointData& operator[](const std::string& key);

  JointStateMap::iterator begin();

  JointStateMap::iterator end();

 protected:
  JointStateMap joint_state_;
}; // end class JointState

} // end namespace labrob

#endif // LABROB_JOINT_STATE_H_