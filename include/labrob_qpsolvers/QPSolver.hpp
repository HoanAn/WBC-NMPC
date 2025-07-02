#ifndef LABROB_QP_SOLVER_HPP_
#define LABROB_QP_SOLVER_HPP_

namespace labrob {
namespace qpsolvers {

template <typename Scalar>
class QPSolver {
 public:
  QPSolver(
      int num_variables,
      int num_equality_constraints,
      int num_inequality_constraints) :
    num_variables_(num_variables),
    num_equality_constraints_(num_equality_constraints),
    num_inequality_constraints_(num_inequality_constraints) {

  }

  virtual void solve(
      const Scalar* H,
      const Scalar* f,
      const Scalar* A,
      const Scalar* b,
      const Scalar* C,
      const Scalar* d_min,
      const Scalar* d_max) = 0;
  virtual const Scalar* get_solution() const = 0;

  const int num_variables_;
  const int num_equality_constraints_;
  const int num_inequality_constraints_;
}; // end class QPSolver

} // end namespace labrob::qpsolvers
} // end namespace labrob

#endif // LABROB_QP_SOLVER_HPP_