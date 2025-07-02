#ifndef LABROB_QPOASES_QP_SOLVER_HPP_
#define LABROB_QPOASES_QP_SOLVER_HPP_

#include "QPSolver.hpp"

// STL
#include <cstring> // memcpy

// qpOASES
#include <qpOASES.hpp>

namespace labrob {
namespace qpsolvers {

template <>
class QPOASESQPSolver : public QPSolver<double>{
 public:
  QPOASESQPSolver(int numVariables, int numEqualityConstraints, int numInequalityConstraints, qpOASES::int_t nWSR = 300) :
  nWSR_(nWSR),
  num_variables_(numVariables),
  num_equality_constraints_(numEqualityConstraints),
  num_inequality_constraints_(numInqualityConstraints) {
    const int numConstraints = numEqualityConstraints + numInequalityConstraints;
    qp_ = qpOASES::QProblem(numVariables, numConstraints);
    qpOASES::Options options;
    options.setToMPC();
    options.printLevel = qpOASES::PL_NONE;
    qp_.setOptions(options);
    H_qpoases_ = (double*) malloc(numVariables * numVariables * sizeof(double));
    f_qpoases_ = (double*) malloc(numVariables * sizeof(double));
    A_qpoases_ = (double*) malloc(numVariables * numConstraints * sizeof(double));
    lbA_qpoases_ = (double*) malloc(numConstraints * sizeof(double));
    ubA_qpoases_ = (double*) malloc(numConstraints * sizeof(double));
    u_ = (double*) malloc(numVariables * sizeof(double));
  }

  ~QPOASESQPSolver() {
    free(H_qpoases_);
    free(f_qpoases_);
    free(A_qpoases_);
    free(lbA_qpoases_);
    free(ubA_qpoases_);
    free(u_);
  }

  // TODO: is there a better way to handle qpOASES different interface?
  void solve(
      double* H,
      double* f,
      double* A,
      double* b,
      double* C,
      double* d_min,
      double* d_max) override {

    constexpr int numConstraints = numEqualityConstraints + numInequalityConstraints;

    // Fill H_qpoases_, f_qpoases_
    memcpy(H_qpoases_, H, numVariables * numVariables * sizeof(double));
    memcpy(f_qpoases_, f, numVariables * sizeof(double));

    // Fill A_qpoases_(r, c):
    for (int r = 0; r < numConstraints; ++r) {
      for (int c = 0; c < numVariables; ++c) {
        if (r < numEqualityConstraints) {
          A_qpoases_[r * numVariables + c] = A[c * numEqualityConstraints + r];
        } else {
          A_qpoases_[r * numVariables + c] = C[c * numInequalityConstraints + (r - numEqualityConstraints)];
        }
      }
    }
    
    // Fill lbA_qpoases_:
    memcpy(lbA_qpoases_, b, numEqualityConstraints * sizeof(double));
    memcpy(lbA_qpoases_ + numEqualityConstraints, d_min, numInequalityConstraints * sizeof(double));
    // Fill ubA_qpoases_:
    memcpy(ubA_qpoases_, b, numEqualityConstraints * sizeof(double));
    memcpy(ubA_qpoases_ + numEqualityConstraints, d_max, numInequalityConstraints * sizeof(double));

    auto nWSR = nWSR_; // Deep copy nWSR to avoid changing original value.
    returnValue_ = qp_.init(
        H_qpoases_, f_qpoases_,
        A_qpoases_, nullptr, nullptr, lbA_qpoases_, ubA_qpoases_, nWSR);

    qp_.getPrimalSolution(u_);
  }

  double* get_solution() const override {
    return u_;
  }

 protected:
  qpOASES::QProblem qp_;
  qpOASES::int_t nWSR_;
  qpOASES::returnValue returnValue_;
  double* H_qpoases_;
  double* f_qpoases_;
  double* A_qpoases_;
  double* lbA_qpoases_;
  double* ubA_qpoases_;
  double* u_;

}; // end class QPOASESQPSolver

} // end namespace qpsolvers
} // end namespace labrob

#endif // LABROB_QPOASES_QP_SOLVER_HPP_