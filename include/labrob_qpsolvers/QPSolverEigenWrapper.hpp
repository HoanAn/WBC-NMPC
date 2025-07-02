#ifndef LABROB_QP_SOLVER_EIGEN_WRAPPER_HPP_
#define LABROB_QP_SOLVER_EIGEN_WRAPPER_HPP_

#include "QPSolver.hpp"

// STL
#include <memory>
#include <iostream>

// Eigen
#include <Eigen/Core>

namespace labrob {
namespace qpsolvers {

template <typename Scalar>
class QPSolverEigenWrapper {
 public:
  QPSolverEigenWrapper(std::shared_ptr<QPSolver<Scalar>> qp_solver_ptr) :
      qp_solver_ptr_(qp_solver_ptr) {

  }

  template <typename DerivedCostH, typename DerivedCostg,
            typename DerivedEqA, typename DerivedEqb,
            typename DerivedIneqC, typename DerivedIneqg>
  void
  solve(
      const Eigen::PlainObjectBase<DerivedCostH>& H,
      const Eigen::PlainObjectBase<DerivedCostg>& g,
      const Eigen::PlainObjectBase<DerivedEqA>& A,
      const Eigen::PlainObjectBase<DerivedEqb>& b,
      const Eigen::PlainObjectBase<DerivedIneqC>& C,
      const Eigen::PlainObjectBase<DerivedIneqg>& lg,
      const Eigen::PlainObjectBase<DerivedIneqg>& ug) {
    qp_solver_ptr_->solve(
        H.data(),
        g.data(),
        A.data(),
        b.data(),
        C.data(),
        lg.data(),
        ug.data()
    );
  }

  Eigen::Matrix<Scalar, Eigen::Dynamic, 1>
  get_solution() {
    const auto* solution_ptr = qp_solver_ptr_->get_solution();
    const int num_variables = qp_solver_ptr_->num_variables_;
    Eigen::Matrix<Scalar, Eigen::Dynamic, 1> solution(num_variables);
    for (int idx = 0; idx < num_variables; ++idx) {
      solution(idx) = *(solution_ptr + idx);
    }
    return solution;
  }

 private:
  std::shared_ptr<QPSolver<Scalar>> qp_solver_ptr_;
}; // end class QPSolverEigenWrapper

} // end namespace qpsolvers
} // end namespace labrob

#endif // LABROB_QP_SOLVER_EIGEN_WRAPPER_HPP_