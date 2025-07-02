// STL
#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

#include <hrp4_locomotion/WalkingManager.hpp>
#include <hrp4_locomotion/JointState.hpp>
#include <hrp4_locomotion/RobotState.hpp>

// ROS
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <XmlRpcValue.h>

// ros_control
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

namespace labrob {

/**
 * \brief IS-MPC controller with a specific hardware interface.
 * \tparam T The hardware interface type used by this controller.
 */
template <class T>
class WalkingController : public controller_interface::Controller<T> {
public:
  /**
   * \brief Initialize the controller from a non-realtime thread with a
   * pointer to the hardware interface.
   * \param hw The specific hardware interface used by this controller.
   * \param nodeHandle A NodeHandle from which the controller should read
   * its configuration.
   */
  bool init(T* hw, ros::NodeHandle& nodeHandle) {

    // Read joint names from ROS server:
    std::string joint_names_param = "/hrp4/walking_controller/joint_names";
    XmlRpc::XmlRpcValue joints_xmlrpc;
    if (!nodeHandle.getParam(joint_names_param, joints_xmlrpc)) {
      ROS_ERROR_STREAM(
          "Cannot read joint names from " << joint_names_param
      );
      return false;
    }

    // Fill joint names:
    if (joints_xmlrpc.getType() != XmlRpc::XmlRpcValue::Type::TypeArray) {
      ROS_ERROR_STREAM(
          "Param read from " << joint_names_param << " is not an array."
      );
      return false;
    }
    for (int i = 0; i < joints_xmlrpc.size(); ++i) {
      if (joints_xmlrpc[i].getType() != XmlRpc::XmlRpcValue::Type::TypeString) {
        ROS_ERROR_STREAM(
            "Param read from " << joint_names_param
                               << " at position " << i << " is not a string."
        );
        return false;
      }
      const std::string& joint_name = joints_xmlrpc[i];
      joint_names_.push_back(joint_name);
    }

    // Get joint handles for all the joints:
    for (const auto& joint_name : joint_names_) {
      joint_handles_[joint_name] = hw->getHandle(joint_name);
    }

    if (!walking_manager_.init(nodeHandle)) {
      return false;
    }

    // Setup odometry publisher (kinematic simulation only):
    odom_publisher_ = nodeHandle.advertise<nav_msgs::Odometry>(
        "/hrp4/ground_truth", 1
    );

    // Define initial robot state (useful for kinematic simulation):
    if (kinematic_simulation_) {
      desired_robot_state_.position = Eigen::Vector3d(-2.1, -1.25, 0.792151);
    }

    return true;
  }

  /**
   * \brief This is called periodically by the realtime thread when the
   * controller is running.
   * \param time The current time.
   * \param period The time passed since the last call to \ref update.
   */
  void update(const ros::Time& time, const ros::Duration& period) {
    // Use desired pose in case of kinematic simulation.
    // TODO: read pose from tf is case of using an actual simulator.
    if (kinematic_simulation_) {
      robot_state_.position = desired_robot_state_.position;
      robot_state_.orientation = desired_robot_state_.orientation;
    }

    // Read state of the joints:
    for (const auto& elem : joint_handles_) {
      const auto& joint_name = elem.first;
      const auto& joint_handle = elem.second;
      robot_state_.joint_state[joint_name].pos = joint_handle.getPosition();
    }

    // WalkingManager:
    walking_manager_.update(robot_state_, desired_robot_state_);

    // Send command to joints:
    for (auto& elem : joint_handles_) {
      const auto& joint_name = elem.first;
      auto& joint_handle = elem.second;
      double cmd = desired_robot_state_.joint_state[joint_name].pos;
      joint_handle.setCommand(cmd);
    }

    // Publish odometry (kinematic simulation only):
    if (kinematic_simulation_) {
      // TODO: add velocity and covariance in both pose and twist.
      nav_msgs::Odometry odom_message;
      odom_message.header.stamp = time;
      odom_message.header.frame_id = "odom";
      odom_message.child_frame_id = "base_link";

      odom_message.pose.pose.position.x = desired_robot_state_.position.x();
      odom_message.pose.pose.position.y = desired_robot_state_.position.y();
      odom_message.pose.pose.position.z = desired_robot_state_.position.z();
      odom_message.pose.pose.orientation.x = desired_robot_state_.orientation.x();
      odom_message.pose.pose.orientation.y = desired_robot_state_.orientation.y();
      odom_message.pose.pose.orientation.z = desired_robot_state_.orientation.z();
      odom_message.pose.pose.orientation.w = desired_robot_state_.orientation.w();

      odom_message.twist.twist.linear.x = desired_robot_state_.linear_velocity.x();
      odom_message.twist.twist.linear.y = desired_robot_state_.linear_velocity.y();
      odom_message.twist.twist.linear.z = desired_robot_state_.linear_velocity.z();
      odom_message.twist.twist.angular.x = desired_robot_state_.angular_velocity.x();
      odom_message.twist.twist.angular.y = desired_robot_state_.angular_velocity.y();
      odom_message.twist.twist.angular.z = desired_robot_state_.angular_velocity.z();

      odom_publisher_.publish(odom_message);
    }
  }

  /**
    * \brief This is called from within the realtime thread just before the
    * first call to \ref update.
    * \param time The current time.
    */
  void starting(const ros::Time& time) {}

  /** \brief This is called from within the realtime thread just after the last
   * update call before the controller is stopped.
   * \param time The current time.
   */
  void stopping(const ros::Time& time) {}

 private:

  //! Joint names.
  std::vector<std::string> joint_names_;

  //! Joint handles of the hardware interface.
  std::unordered_map<std::string, hardware_interface::JointHandle> joint_handles_;

  //! Walking manager:
  labrob::WalkingManager walking_manager_;

  //! Robot state:
  labrob::RobotState robot_state_;

  //! Desired robot state:
  labrob::RobotState desired_robot_state_;

  //! Odometry publisher for kinematic simulation:
  ros::Publisher odom_publisher_;

  bool kinematic_simulation_ = true;


}; // end class WalkingController

} // end namespace labrob

PLUGINLIB_EXPORT_CLASS(
    labrob::WalkingController<hardware_interface::PositionJointInterface>,
    controller_interface::ControllerBase)