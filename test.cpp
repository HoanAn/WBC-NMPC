// std
#include <fstream>
#include <memory>
#include <vector>
#include <chrono> 

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <hrp4_locomotion/WalkingManager.hpp>
#include <labrob_qpsolvers/qpsolvers.hpp>

// Pinocchio
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea-derivatives.hpp>
#include <pinocchio/autodiff/casadi.hpp>

#include <hrp4_locomotion/GaitConfiguration.hpp>
#include <hrp4_locomotion/JointCommand.hpp>
#include <hrp4_locomotion/RobotState.hpp>

#include <casadi/casadi.hpp>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace casadi;
using RobotModel = pinocchio::ModelTpl<double>;
using RobotData = pinocchio::DataTpl<double>;
using CasadiModel = pinocchio::ModelTpl<casadi::MX>;
using CasadiData = pinocchio::DataTpl<casadi::MX>;



std::shared_ptr<labrob::qpsolvers::QPSolverEigenWrapper<double>> qp_solver_ptr_;

int main() {
    int num_state = 3;
    int num_contrl = 1;
    int num_variables = num_state + num_contrl;
    int num_eq_constraints = 1;
    int num_ineq_constraints = 2;

    Eigen::MatrixXd H(num_variables, num_variables);
    Eigen::VectorXd f(num_variables);
    Eigen::MatrixXd A(num_eq_constraints, num_variables);
    Eigen::VectorXd b(num_eq_constraints);
    Eigen::MatrixXd C(num_ineq_constraints, num_variables);
    Eigen::VectorXd d_min(num_ineq_constraints);
    Eigen::VectorXd d_max(num_ineq_constraints);

    H << 6, 0, 0, 0,
         0, 4, 0, 0,
         0, 0, 7, 0,
         0, 0, 0, 9;

    f << 1, 2, 3, 4;
    A << 4, 0, 0, 0;
    b << 1;
    C << 1, 1, 1, 1,
         2, 2, 2, 2;
    d_min << 0, 0;
    d_max << 2, 2;
        
    qp_solver_ptr_ = std::make_shared<labrob::qpsolvers::QPSolverEigenWrapper<double>>(
      std::make_shared<labrob::qpsolvers::OSQPSolver>(num_variables, num_eq_constraints, num_ineq_constraints));

    qp_solver_ptr_->solve(
        H, f, A, b, C, d_min, d_max);

    std::cerr<< "Solution: " << std::endl << qp_solver_ptr_->get_solution()<< std::endl;

    
    
}