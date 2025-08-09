#include <hrp4_locomotion/utils.hpp>

// STL
#include <fstream>
#include <iostream>
#include <termios.h>
#include <unistd.h>

// Boost
#include <boost/algorithm/string.hpp>

// Eigen
#include <Eigen/Geometry>

// hrp4_locomotion
#include <hrp4_locomotion/SE3.hpp>


namespace labrob {

Eigen::Matrix<double, 6, 1>
err_frameplacement(const pinocchio::SE3& Ta, const pinocchio::SE3& Tb) {
  // TODO: how do you use pinocchio::log6?
  Eigen::Matrix<double, 6, 1> err;
  err << err_translation(Ta.translation(), Tb.translation()),
      err_rotation(Ta.rotation(), Tb.rotation());
  return err;
}

Eigen::Vector3d
err_translation(const Eigen::Vector3d& pa, const Eigen::Vector3d& pb) {
  return pa - pb;
}

Eigen::Vector3d
err_rotation(const Eigen::Matrix3d& Ra, const Eigen::Matrix3d& Rb) {
  // TODO: how do you use pinocchio::log3?
  Eigen::Matrix3d Rdiff = Rb.transpose() * Ra;
  auto aa = Eigen::AngleAxisd(Rdiff);
  return aa.angle() * Ra * aa.axis();
}

Eigen::VectorXd
robot_state_to_pinocchio_joint_configuration(
    const pinocchio::Model& robot_model,
    const labrob::RobotState& robot_state
) {
  // labrob::RobotState representation to Pinocchio representation:
  // TODO: RobotState also has information about the velocity of the floating base.
  // TODO: is there a less error-prone way to convert representation?
  Eigen::VectorXd q(robot_model.nq);
  q.head<3>() = robot_state.position;
  q.segment<4>(3) = robot_state.orientation.coeffs();
  // NOTE: start from joint id (2) to skip frames "universe" and "root_joint".
  for(pinocchio::JointIndex joint_id = 2;
      joint_id < (pinocchio::JointIndex) robot_model.njoints;
      ++joint_id) {
    const auto& joint_name = robot_model.names[joint_id];
    q[joint_id + 5] = robot_state.joint_state[joint_name].pos;
  }

  return q;
}

Eigen::VectorXd
robot_state_to_pinocchio_joint_velocity(
    const pinocchio::Model& robot_model,
    const labrob::RobotState& robot_state
) {
  Eigen::VectorXd qdot(robot_model.nv);
  qdot.head<3>() = robot_state.linear_velocity;
  qdot.segment<3>(3) = robot_state.angular_velocity;
  // NOTE: start from joint id (2) to skip frames "universe" and "root_joint".
  for(pinocchio::JointIndex joint_id = 2;
      joint_id < (pinocchio::JointIndex) robot_model.njoints;
      ++joint_id) {
    const auto& joint_name = robot_model.names[joint_id];
    qdot[joint_id + 4] = robot_state.joint_state[joint_name].vel;
  }
  
  return qdot;
}

Eigen::Vector3d
updateState(
    const LIPState& lip_state,
    double zmpDot,
    int dim,
    double com_target_height,
    int64_t control_timestep_msec
) {

  double control_timestep = 0.001 * static_cast<double>(control_timestep_msec);

  double omega = std::sqrt(9.81 / com_target_height);

  // Update the state along the dim-th direction (0,1,2) = (x,y,z)

  double ch = cosh(omega * control_timestep);
  double sh = sinh(omega * control_timestep);

  Eigen::Matrix3d A_upd = Eigen::Matrix3d::Zero();
  Eigen::Vector3d B_upd = Eigen::Vector3d::Zero();
  A_upd<<ch,sh/omega,1-ch,omega*sh,ch,-omega*sh,0,0,1;
  B_upd<<control_timestep-sh/omega,1-ch,control_timestep;

  Eigen::Vector3d currentState = Eigen::Vector3d(
      lip_state.com_pos_(dim),
      lip_state.com_vel_(dim),
      lip_state.zmp_pos_(dim)
  );

  if (dim == 2) return A_upd*(currentState + Eigen::Vector3d(0.0,0.0,com_target_height)) + B_upd*zmpDot - Eigen::Vector3d(0.0,0.0,com_target_height);

  return A_upd*currentState + B_upd*zmpDot;
}

labrob::WalkingState
walkingStateFromString(
    const std::string& walking_state_str
) {
  labrob::WalkingState walking_state;
  if (walking_state_str == "Init") {
    walking_state = labrob::WalkingState::Init;
  } else if (walking_state_str == "PostureRegulation") {
    walking_state = labrob::WalkingState::PostureRegulation;
  } else if (walking_state_str == "Standing") {
    walking_state = labrob::WalkingState::Standing;
  } else if (walking_state_str == "Starting") {
    walking_state = labrob::WalkingState::Starting;
  } else if (walking_state_str == "SingleSupport") {
    walking_state = labrob::WalkingState::SingleSupport;
  } else if (walking_state_str == "DoubleSupport") {
    walking_state = labrob::WalkingState::DoubleSupport;
  } else if (walking_state_str == "Stopping") {
    walking_state = labrob::WalkingState::Stopping;
  } else if (walking_state_str == "AbortStarting") {
    walking_state = labrob::WalkingState::AbortStarting;
  } else if (walking_state_str == "AbortWalking") {
    walking_state = labrob::WalkingState::AbortWalking;
  }
  return walking_state;
}

bool readArgosFootstepPlan(
    const std::string& file_path,
    std::deque<labrob::FootstepPlanElement>& footstep_plan
) {
  // Hparams (const single and double support duration in WoS RAS23);
  // durations are expressed in [ms].
  int64_t init_duration = 4400;
  int64_t starting_duration = 600;
  int64_t single_support_duration = 600;
  int64_t double_support_duration = 400;
  int64_t stopping_duration = 400;
  int64_t standing_duration = 2000;

  // Open the file:
  std::ifstream input_file_stream(file_path);
  if (!input_file_stream.is_open()) {
    std::cerr << "Cannot open file" << std::endl;
    return false;
  }
  footstep_plan.clear();
  // Read the file and process each line one by one (skip frame_id):
  std::string frame_id;
  std::getline(input_file_stream, frame_id);
  bool first_element_read = false;
  bool starting_phase_read = false;
  std::string line;
  while (std::getline(input_file_stream, line)) {
    std::cerr << "Reading line" << std::endl;
    // Split line into tokens:
    std::vector<std::string> line_splitted;
    boost::split(line_splitted, line, boost::is_any_of(","));
    std::string qL_x_str = line_splitted[0];
    std::string qL_y_str = line_splitted[1];
    std::string qL_z_str = line_splitted[2];
    std::string qL_w_str = line_splitted[3];
    std::string qR_x_str = line_splitted[4];
    std::string qR_y_str = line_splitted[5];
    std::string qR_z_str = line_splitted[6];
    std::string qR_w_str = line_splitted[7];
    std::string supp_str = line_splitted[8];
    std::string h_z_str  = line_splitted[9];
    // Convert tokens into data:
    double qL_x = std::stod(qL_x_str);
    double qL_y = std::stod(qL_y_str);
    double qL_z = std::stod(qL_z_str);
    double qL_w = std::stod(qL_w_str);
    double qR_x = std::stod(qR_x_str);
    double qR_y = std::stod(qR_y_str);
    double qR_z = std::stod(qR_z_str);
    double qR_w = std::stod(qR_w_str);
    labrob::Foot support_foot = (
        supp_str == "LEFT" ? labrob::Foot::LEFT : labrob::Foot::RIGHT
    );
    double h_z  = std::stod(h_z_str);
    // Add single support element to footstep plan:
    if (first_element_read && starting_phase_read) {
      auto previous_element = footstep_plan.back();
      footstep_plan.push_back(labrob::FootstepPlanElement(
          labrob::DoubleSupportConfiguration(
              previous_element.getFeetPlacement().getLeftFootConfiguration(),
              previous_element.getFeetPlacement().getRightFootConfiguration(),
              (support_foot == labrob::Foot::LEFT ? labrob::Foot::RIGHT : labrob::Foot::LEFT)
          ),
          h_z,
          single_support_duration,
          labrob::WalkingState::SingleSupport
      ));
      // Add double support element to footstep plan:
      footstep_plan.push_back(labrob::FootstepPlanElement(
          labrob::DoubleSupportConfiguration(
              labrob::SE3(labrob::Rz<double>(qL_w), Eigen::Vector3d(qL_x, qL_y, qL_z)),
              labrob::SE3(labrob::Rz<double>(qR_w), Eigen::Vector3d(qR_x, qR_y, qR_z)),
              (support_foot == labrob::Foot::LEFT ? labrob::Foot::RIGHT : labrob::Foot::LEFT)
          ),
          0.0,
          double_support_duration,
          labrob::WalkingState::DoubleSupport
      ));
    } else if (first_element_read && !starting_phase_read) {
      starting_phase_read = true;
      auto previous_element = footstep_plan.back();
      footstep_plan.push_back(labrob::FootstepPlanElement(
          previous_element.getFeetPlacement(),
          h_z,
          single_support_duration,
          labrob::WalkingState::SingleSupport
      ));
      // Add double support element to footstep plan:
      footstep_plan.push_back(labrob::FootstepPlanElement(
          labrob::DoubleSupportConfiguration(
              labrob::SE3(labrob::Rz<double>(qL_w), Eigen::Vector3d(qL_x, qL_y, qL_z)),
              labrob::SE3(labrob::Rz<double>(qR_w), Eigen::Vector3d(qR_x, qR_y, qR_z)),
              previous_element.getFeetPlacement().getSupportFoot()
          ),
          0.0,
          double_support_duration,
          labrob::WalkingState::DoubleSupport
      ));
    } else {
      first_element_read = true;
      // Add init element to footstep plan:
      footstep_plan.push_back(labrob::FootstepPlanElement(
          labrob::DoubleSupportConfiguration(
              labrob::SE3(labrob::Rz<double>(qL_w), Eigen::Vector3d(qL_x, qL_y, qL_z)),
              labrob::SE3(labrob::Rz<double>(qR_w), Eigen::Vector3d(qR_x, qR_y, qR_z)),
              support_foot
          ),
          0.0,
          init_duration,
          labrob::WalkingState::Init
      ));
      // Add starting element to footstep plan:
      footstep_plan.push_back(labrob::FootstepPlanElement(
          labrob::DoubleSupportConfiguration(
              labrob::SE3(labrob::Rz<double>(qL_w), Eigen::Vector3d(qL_x, qL_y, qL_z)),
              labrob::SE3(labrob::Rz<double>(qR_w), Eigen::Vector3d(qR_x, qR_y, qR_z)),
              support_foot
          ),
          0.0,
          starting_duration,
          labrob::WalkingState::Starting
      ));
    }
  }

  // Replace last DoubleSupport with Stopping and Standing phases:
  auto& stopping_element = footstep_plan.back();
  stopping_element.setSwingFootTrajectoryHeight(0.0);
  stopping_element.setDuration(stopping_duration);
  stopping_element.setWalkingState(labrob::WalkingState::Stopping);
  auto standing_element = footstep_plan.back();
  standing_element.setSwingFootTrajectoryHeight(0.0);
  standing_element.setDuration(standing_duration);
  standing_element.setWalkingState(labrob::WalkingState::Standing);
  footstep_plan.push_back(standing_element);

  return true;
}

bool readHumanoids2023FootstepPlan(
    const std::string& file_path,
    const labrob::DoubleSupportConfiguration& current_feet_placement,
    const labrob::WalkingState& current_walking_state,
    std::deque<labrob::FootstepPlanElement>& footstep_plan
) {
  // Hparams:
  int64_t init_duration = 4400;
  // Open the file:
  std::ifstream input_file_stream(file_path);
  if (!input_file_stream.is_open()) {
    return false;
  }
  footstep_plan.clear();
  // Add initial Init element in case the current walking state is Init:
  if (current_walking_state == labrob::WalkingState::Init) {
    footstep_plan.push_back(labrob::FootstepPlanElement(
        current_feet_placement,
        0.0,
        init_duration,
        labrob::WalkingState::Init
    ));
  }
  // Read the file and process each line one by one:
  // NOTE: assuming initial configuration of the footstep plan coincides with
  //       current feet placement.
  std::string line;
  while (std::getline(input_file_stream, line)) {
    // Split line into tokens:
    std::stringstream ss(line);
    std::string qLeft_x_str;
    std::string qLeft_y_str;
    std::string qLeft_z_str;
    std::string qLeft_theta_str;
    std::string qRight_x_str;
    std::string qRight_y_str;
    std::string qRight_z_str;
    std::string qRight_theta_str;
    std::string support_foot_str;
    std::string duration_str;
    std::string walking_state_str;
    ss >> qLeft_x_str >> qLeft_y_str >> qLeft_z_str >> qLeft_theta_str
        >> qRight_x_str >> qRight_y_str >> qRight_z_str >> qRight_theta_str
        >> duration_str >> support_foot_str >> walking_state_str;
    // Convert tokens into data:
    auto left_foot_configuration = labrob::SE3(
        labrob::Rz(std::stod(qLeft_theta_str)),
        Eigen::Vector3d(std::stod(qLeft_x_str), std::stod(qLeft_y_str), std::stod(qLeft_z_str))
    );
    auto right_foot_configuration = labrob::SE3(
        labrob::Rz(std::stod(qRight_theta_str)),
        Eigen::Vector3d(std::stod(qRight_x_str), std::stod(qRight_y_str), std::stod(qRight_z_str))
    );
    labrob::Foot support_foot = labrob::Foot::LEFT;
    if (support_foot_str == "RIGHT") {
      support_foot = labrob::Foot::RIGHT;
    }
    double qSupp_duration = std::stod(duration_str);
    labrob::WalkingState walking_state = walkingStateFromString(walking_state_str);
    int64_t duration_ms = 1000 * qSupp_duration;
    double h_z = 0.0;
    footstep_plan.push_back(labrob::FootstepPlanElement(
        labrob::DoubleSupportConfiguration(
            left_foot_configuration, right_foot_configuration, support_foot
        ),
        h_z,
        duration_ms,
        walking_state
    ));
  }

  // Choose correct swing foot trajectory height:
  // TODO: this part should be completely replaced by first reading regions and
  //       then generating swing foot trajectory that are collision-free wrt
  //       regions themselves (or by integrating swing foot trajectory
  //       generation into footstep planner presented in Humanoids 2023).
  // NOTE: the following assumes the robot is only going up the stairs.
  double delta_z_threshold = 1e-3;
  double h_z_same_patch = 0.02;
  double h_z_going_up = 0.16;
  for (auto it = footstep_plan.begin(); it != footstep_plan.end(); ++it) {
    auto& footstep_plan_elem = *it;
    // NOTE: assuming a SingleSupport element is always followed by another one.
    if (footstep_plan_elem.getWalkingState() == labrob::WalkingState::SingleSupport) {
      auto& next_footstep_plan_elem = *(it + 1);
      double delta_z_swg = 
          next_footstep_plan_elem.getFeetPlacement().getSwingFootConfiguration().p.z() -
          footstep_plan_elem.getFeetPlacement().getSwingFootConfiguration().p.z();
      if (std::abs(delta_z_swg) < delta_z_threshold) {
        footstep_plan_elem.setSwingFootTrajectoryHeight(h_z_same_patch);
      } else {
        double delta_z_wrt_supp =
            next_footstep_plan_elem.getFeetPlacement().getSwingFootConfiguration().p.z() -
            next_footstep_plan_elem.getFeetPlacement().getSupportFootConfiguration().p.z();
        if (delta_z_wrt_supp < 1e-3) {
          // Support foot on patch lower wrt swing foot:
          footstep_plan_elem.setSwingFootTrajectoryHeight(h_z_going_up - std::abs(delta_z_swg));
        } else {
          // Support foot on same patch wrt swing foot:
          footstep_plan_elem.setSwingFootTrajectoryHeight(h_z_going_up);
        }
      }
    }
  }

  return true;
}

bool readFootstepPlan(
    const std::string& file_path,
    std::deque<labrob::FootstepPlanElement>& footstep_plan
) {
  // Open the file:
  std::ifstream input_file_stream(file_path);
  if (!input_file_stream.is_open()) {
    return false;
  }
  footstep_plan.clear();
  // Read the file and process each line one by one:
  std::string line;
  while (std::getline(input_file_stream, line)) {
    // Split line into tokens:
    std::vector<std::string> line_splitted;
    boost::split(line_splitted, line, boost::is_any_of(","));
    std::string qL_px_str = line_splitted[ 0];
    std::string qL_py_str = line_splitted[ 1];
    std::string qL_pz_str = line_splitted[ 2];
    std::string qL_qx_str = line_splitted[ 3];
    std::string qL_qy_str = line_splitted[ 4];
    std::string qL_qz_str = line_splitted[ 5];
    std::string qL_qw_str = line_splitted[ 6];
    std::string qR_px_str = line_splitted[ 7];
    std::string qR_py_str = line_splitted[ 8];
    std::string qR_pz_str = line_splitted[ 9];
    std::string qR_qx_str = line_splitted[10];
    std::string qR_qy_str = line_splitted[11];
    std::string qR_qz_str = line_splitted[12];
    std::string qR_qw_str = line_splitted[13];
    std::string supp_str  = line_splitted[14];
    std::string h_z_str   = line_splitted[15];
    std::string duration_str  = line_splitted[16];
    std::string walking_state_str = line_splitted[17];
    // Convert tokens into data:
    double qL_px = std::stod(qL_px_str);
    double qL_py = std::stod(qL_py_str);
    double qL_pz = std::stod(qL_pz_str);
    double qL_qx = std::stod(qL_qx_str);
    double qL_qy = std::stod(qL_qy_str);
    double qL_qz = std::stod(qL_qz_str);
    double qL_qw = std::stod(qL_qw_str);
    double qR_px = std::stod(qR_px_str);
    double qR_py = std::stod(qR_py_str);
    double qR_pz = std::stod(qR_pz_str);
    double qR_qx = std::stod(qR_qx_str);
    double qR_qy = std::stod(qR_qy_str);
    double qR_qz = std::stod(qR_qz_str);
    double qR_qw = std::stod(qR_qw_str);
    labrob::Foot support_foot = (
        supp_str == "LEFT" ? labrob::Foot::LEFT : labrob::Foot::RIGHT
    );
    double h_z  = std::stod(h_z_str);
    double duration = std::stod(duration_str);
    labrob::WalkingState walking_state = walkingStateFromString(walking_state_str);
    // Add element to footstep plan:
    footstep_plan.push_back(labrob::FootstepPlanElement(
        labrob::DoubleSupportConfiguration(
            labrob::SE3(Eigen::Quaterniond(qL_qw, qL_qx, qL_qy, qL_qz).toRotationMatrix(), Eigen::Vector3d(qL_px, qL_py, qL_pz)),
            labrob::SE3(Eigen::Quaterniond(qR_qw, qR_qx, qR_qy, qR_qz).toRotationMatrix(), Eigen::Vector3d(qR_px, qR_py, qR_pz)),
            support_foot
        ),
        h_z,
        duration,
        walking_state
    ));
  }
  return true;
}

void saveFootstepPlan(
    const std::deque<labrob::FootstepPlanElement>& footstep_plan,
    const std::string& file_path
) {
  std::ofstream output_file_stream(file_path);
  for (const auto& footstep_plan_element : footstep_plan) {
    const auto& feet_placement = footstep_plan_element.getFeetPlacement();
    double h_z = footstep_plan_element.getSwingFootTrajectoryHeight();
    double duration = footstep_plan_element.getDuration();
    const auto& walking_state = footstep_plan_element.getWalkingState();
    const auto& qLeft = feet_placement.getLeftFootConfiguration();
    const auto& qRight = feet_placement.getRightFootConfiguration();
    const auto& qLeft_p = qLeft.p;
    const auto qLeft_q = Eigen::Quaterniond(qLeft.R);
    const auto& qRight_p = qRight.p;
    const auto qRight_q = Eigen::Quaterniond(qRight.R);
    const auto& support_foot = feet_placement.getSupportFoot();
    std::string support_foot_str = (
        support_foot == labrob::Foot::LEFT ? "LEFT" : "RIGHT"
    );
    output_file_stream << qLeft_p.x() << ","
        << qLeft_p.y() << ","
        << qLeft_p.z() << ","
        << qLeft_q.x() << ","
        << qLeft_q.y() << ","
        << qLeft_q.z() << ","
        << qLeft_q.w() << ","
        << qRight_p.x() << ","
        << qRight_p.y() << ","
        << qRight_p.z() << ","
        << qRight_q.x() << ","
        << qRight_q.y() << ","
        << qRight_q.z() << ","
        << qRight_q.w() << ","
        << support_foot_str << ","
        << h_z << ","
        << duration << ","
        << labrob::to_string(walking_state) << std::endl;
  }
}

void pressAnyKey() {
    // Disable canonical mode and echo for terminal input
    termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    std::cout << "Press the spacebar to continue..." << std::endl;

    char ch;
    // Loop until the spacebar is pressed
    do {
        ch = getchar();
    } while (ch != ' ');

    // Restore terminal settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
}

} // end namespace labrob