#ifndef LABROB_OS_QP_SOLVER_HPP_
#define LABROB_OS_QP_SOLVER_HPP_

#include "QPSolver.hpp"
#include <Eigen/Sparse>
// OSQP
#include "osqp.h"
#include <iostream>
#include <casadi/casadi.hpp>

#include "hrp4_locomotion/eval_codegen_func.h"
#include "hrp4_locomotion/utils.hpp"



namespace labrob {
namespace qpsolvers {


class OSQPSolver : public QPSolver<double> {
 public:
  OSQPSolver(int numVariables, int numEqualityConstraints, int numInequalityConstraints,CSCMatrix_params& P_params) :
    QPSolver<double>(numVariables, numEqualityConstraints, numInequalityConstraints)
  {
    const int numConstraints = numEqualityConstraints + numInequalityConstraints;
    num_var_ = numVariables;
    num_constr_ = numConstraints;
    setup_flag_ = 1; // After a successful executtion (no errors), flags are reset to 0
    update_flag_ = 1;
    solve_flag_ = 1;
    qp_setting_ = OSQPSettings_new();
    qp_setting_->polishing = false;
    qp_setting_->max_iter = 500;                
    qp_setting_->warm_starting = true;
    qp_setting_->eps_abs = 1e-2;  // Default is 1e-3
    qp_setting_->eps_rel = 1e-2;  // Default is 1e-3
    qp_setting_->eps_prim_inf = 1e-3;  // Primal infeasibility tolerance
    qp_setting_->eps_dual_inf = 1e-3;  // Dual infeasibility tolerance
    qp_setting_->adaptive_rho = false;

    //P_ = OSQPCscMatrix_identity(numVariables); // Initialize an identity matrix for P
    P_ = OSQPCscMatrix_new(P_params.nrows, P_params.ncols, P_params.nzeros,
                           P_params.data.data(), P_params.row_indices.data(), P_params.col_pointers.data());

    //CSCMatrix_params csc_params_A_init = denseToCSC_param(A_init.data(), numConstraints, numVariables);
    // OSQp fix the sparsity structure while runtime solving. Thus, this code initlize the fixed sparsity structure
    // for the solver's internal matrices. The data is taken from the Casadi codegen function.
    CSCMatrix_params csc_params_A_init;
    const casadi_int * A_sparsity = Jacob_f_total_constraint_sparsity_out(0);
    const casadi_int * A_colind = A_sparsity + 2;

    csc_params_A_init.nrows = A_sparsity[0];
    csc_params_A_init.ncols = A_sparsity[1];
    csc_params_A_init.nzeros = A_sparsity[csc_params_A_init.ncols + 2];
    
    csc_params_A_init.row_indices.resize(csc_params_A_init.nzeros);
    csc_params_A_init.col_pointers.resize(csc_params_A_init.ncols + 1);

    csc_params_A_init.data.resize(csc_params_A_init.nzeros);

    for (casadi_int i = 0; i <= csc_params_A_init.ncols; ++i){
      csc_params_A_init.col_pointers[i] = A_colind[i];
    }

    const casadi_int* A_rowind = A_colind + csc_params_A_init.ncols+1;


    for (casadi_int i = 0; i < csc_params_A_init.nzeros; ++i){
      csc_params_A_init.row_indices[i] = A_rowind[i];  
      csc_params_A_init.data[i] = 1.0; // Just for initializing, set all data to 1.0
      if (i == 0 ) std::cout << "A_rowind [0]: "<< A_rowind[i] << std::endl;
      if (i == csc_params_A_init.nzeros -1 ) std::cout << "A_rowind [" << i << "]: " << A_rowind[i] << std::endl; 
    }
    // For convert the codegen data, refer to this: https://github.com/casadi/casadi/issues/3701


    std::cerr << "Row paramA_: " << csc_params_A_init.nrows << std::endl;
    std::cerr << "Col paramA_: " << csc_params_A_init.ncols << std::endl;
    std::cerr << "Nzeros paramA_: " << csc_params_A_init.nzeros << std::endl;
  
    
    A_ = OSQPCscMatrix_new(csc_params_A_init.nrows, csc_params_A_init.ncols,csc_params_A_init.nzeros,
                           csc_params_A_init.data.data(), csc_params_A_init.row_indices.data(), csc_params_A_init.col_pointers.data());

    std::cerr << "Done initilize A_"<< std::endl;
    std::cerr << "Row A_: " << A_->m << std::endl;
    std::cerr << "Col A_: " << A_->n << std::endl;
    std::cerr << "Nzeros A_: " << A_->nzmax << std::endl;
    std::cerr << "Done initilize A_ with size: " << numConstraints << "x" << numVariables << std::endl;
   
    q_  = (OSQPFloat*) malloc(numVariables * sizeof(OSQPFloat)); 
    
    
    for (int i = 0; i < numVariables; ++i) {
      q_[i] = 0.0; // Initialize q to zero
    }

    lower_ = (OSQPFloat*) malloc(numConstraints * sizeof(OSQPFloat));
    upper_ = (OSQPFloat*) malloc(numConstraints * sizeof(OSQPFloat));

    for (int i = 0; i < numConstraints; ++i) {
      lower_[i] = 0; // Initialize lower bounds to -infinity
      upper_[i] = 1;  // Initialize upper bounds to infinity
    }

    // Initialize the OSQP solver
    std::cerr << "Setting up QP solver"<< std::endl;
    solve_flag_ = osqp_setup(&qp_solver_, 
                              P_, q_, A_, lower_, upper_, 
                              numConstraints, numVariables, qp_setting_);


    if (solve_flag_ != 0) {
      std::cerr << "OSQP setup failed with error code: " << solve_flag_ << std::endl;
      exit(EXIT_FAILURE);
    }
    std::cerr << "Done setting up QP solver"<< std::endl;
    u_ = (double*) malloc(numVariables * sizeof(double));
  
  }

  ~OSQPSolver() {
    free(u_);
    free(q_);
    free(lower_);
    free(upper_);
    OSQPCscMatrix_free(A_);
    OSQPCscMatrix_free(P_);

  }

  void solve(
      const double* H,
      const double* f,
      const double* A,
      const double* b,
      const double* C,
      const double* d_min,
      const double* d_max) override {

    

    // Convert H (const double*) to a std::vector<std::vector<double>>
    std::cerr << "Extracting H param: "<< std::endl;
    CSCMatrix_params csc_params_H = denseToCSC_param(H, num_var_, num_var_);
    std::cerr << "Done Extract H param: "<< std::endl;

    std::cout << "num_constr_: " << num_constr_ << std::endl;
    std::cout << "num_equality_constraints_: " << num_equality_constraints_ << std::endl;
    std::cout << "num_inequality_constraints_: " << num_inequality_constraints_ << std::endl;
    // Also check the size of the source vectors
    std::cout << "Size of b: " << *b << std::endl;
    std::cout << "Size of d_min: " << *d_min << std::endl;

    Eigen::MatrixXd A_OSQP (num_constr_, num_var_);
    A_OSQP.block(0,0, num_equality_constraints_, num_var_) = Eigen::Map<const Eigen::MatrixXd>(A, num_equality_constraints_, num_var_);
    
    A_OSQP.block(num_equality_constraints_, 0, num_inequality_constraints_, num_var_) = Eigen::Map<const Eigen::MatrixXd>(C, num_inequality_constraints_, num_var_);
    std::cerr << "Data A_OSQP_: ";
    for(int i = 0; i < num_constr_*num_var_; ++i) {
      std::cerr << "A_OSQP_[" << i << "] = " << A_OSQP.data()[i] << std::endl;
    }
    
    CSCMatrix_params csc_params_A = denseToCSC_param(A_OSQP.data(), num_constr_, num_var_);


    std::cerr << "Data paramA_: ";
    for(int i = 0; i < csc_params_A.nzeros; ++i) {
      std::cerr << "paramA_[" << i << "] = " << csc_params_A.data[i] << std::endl;
    }
    
    for(int i = 0; i < csc_params_A.nzeros; ++i) {
      std::cerr << "param A_row[" << i << "] = " << csc_params_A.row_indices[i] << std::endl;
    }
    
    for(int i = 0; i < num_var_+1; ++i) {
      std::cerr << "param A_colp[" << i << "] = " << csc_params_A.col_pointers[i] << std::endl;
    }

    Eigen::MatrixXd d_min_OSQP(num_constr_,1);
    d_min_OSQP.block(0, 0, num_equality_constraints_, 1) = Eigen::Map<const Eigen::MatrixXd>(b, num_equality_constraints_, 1);
    d_min_OSQP.block(num_equality_constraints_, 0, num_inequality_constraints_, 1) = Eigen::Map<const Eigen::MatrixXd>(d_min, num_inequality_constraints_, 1);

    for(int i = 0; i < num_constr_; ++i) {
      std::cerr << "dmin_OSQP_[" << i << "] = " << d_min_OSQP.data()[i] << std::endl;
    }

    Eigen::MatrixXd d_max_OSQP(num_constr_,1);
    d_max_OSQP.block(0, 0, num_equality_constraints_, 1) = Eigen::Map<const Eigen::MatrixXd>(b, num_equality_constraints_, 1);
    d_max_OSQP.block(num_equality_constraints_, 0, num_inequality_constraints_, 1) = Eigen::Map<const Eigen::MatrixXd>(d_max, num_inequality_constraints_, 1);

    // Update OSQPCscMatrix for P and A
    OSQPInt update_mat_flag = osqp_update_data_mat(qp_solver_,
                                        csc_params_H.data.data(),OSQP_NULL,csc_params_H.nzeros,
                                        csc_params_A.data.data(),csc_params_A.row_indices.data(),csc_params_A.nzeros);

    OSQPInt update_vec_flag = osqp_update_data_vec(qp_solver_,
                                        (OSQPFloat*) f, (OSQPFloat*) d_min_OSQP.data(), (OSQPFloat*) d_max_OSQP.data());                                       

   
    if (update_mat_flag != 0 || update_vec_flag != 0) {
      std::cerr << "OSQP update failed with error code: " << update_mat_flag << ", " << update_vec_flag << std::endl;
      exit(EXIT_FAILURE);
    }
    // Solve OSQP problem
    std::cerr << "Solver info: "<< qp_solver_->info << std::endl;
    OSQPInt solve_flag_ = osqp_solve(qp_solver_);

    if( solve_flag_ != 0) {
      std::cerr << "OSQP solve failed with error code: " << solve_flag_ << std::endl;
      exit(EXIT_FAILURE);
    }

    u_ = qp_solver_->solution->x; // Get the solution vector

    // free_csc_params(csc_params_H);
    // free_csc_params(csc_params_A);

  
  }

  void solve_CCS( 
    const CSCMatrix_params& P,
    const double * f,
    const CSCMatrix_params& A,
    const double* d_min,
    const double* d_max) override {


    // Update OSQPCscMatrix for P and A
    //const CSCMatrix_params P_new = *P;
    OSQPInt update_mat_flag = osqp_update_data_mat(qp_solver_,
                                        P.data.data(),OSQP_NULL,P.nzeros,
                                        A.data.data(),OSQP_NULL,A.nzeros);

    OSQPInt update_vec_flag = osqp_update_data_vec(qp_solver_,
                                        (OSQPFloat*) f, (OSQPFloat*) d_min, (OSQPFloat*) d_max);                                       

   
    if (update_mat_flag != 0 || update_vec_flag != 0) {
      std::cerr << "OSQP update failed with error code: " << update_mat_flag << ", " << update_vec_flag << std::endl;
      exit(EXIT_FAILURE);
    }
    // Solve OSQP problem
    std::cerr << "Solver info: "<< qp_solver_->info << std::endl;
    OSQPInt solve_flag_ = osqp_solve(qp_solver_);

    if( solve_flag_ != 0) {
      std::cerr << "OSQP solve failed with error code: " << solve_flag_ << std::endl;
      exit(EXIT_FAILURE);
    }

    u_ = qp_solver_->solution->x; // Get the solution vector

    // free_csc_params(csc_params_H);
    // free_csc_params(csc_params_A);




    };

  

  const double* get_solution() const override {
    return u_;
  }

  //OSQPCscMatrix *P_;// Pointer to the CSC matrix for the quadratic term in the cost function
  
  // void update_weight_terms( WholeBodyMPCParams params ){

  // };

 private:
  // Define a structure for the CSC matrix
//   struct CSCMatrix_params_qp {
//     OSQPInt nrows;              // Number of rows
//     OSQPInt ncols;              // Number of columns
//     OSQPInt nzeros;             // Non-zero values
//     OSQPFloat* data;            // Vector of data
//     OSQPInt*   row_indices;     // Vector of row indices
//     OSQPInt*   col_pointers;    // Column pointers (start of each column in values array)
//   };

 
//   // Function to convert a dense matrix to CSC format and create an OSQPCscMatrix
//   CSCMatrix_params_qp denseToCSC_param(const double* H, int nrows, int ncols) {
//     // Vectors to store CSC data
//     CSCMatrix_params_qp params;
//     std::cerr << "Extracting CSC params: "<< std::endl;
//     params.nrows = nrows;
//     params.ncols = ncols;

//     std::vector<double> values;     // Non-zero values
//     std::vector<int> row_indices;  // Row indices of non-zero values
//     std::vector<int> col_pointers(ncols + 1, 0); // Column pointers

//     // Iterate through the dense matrix to populate CSC arrays
//     for (int col = 0; col < ncols; ++col) {
//         col_pointers[col] = values.size(); // Start of the current column
//         for (int row = 0; row < nrows; ++row) {
//             double value = H[row + col * nrows]; // Access element in column-major order
//             if (value != 0.0) {
//                 values.push_back(value);       // Store non-zero value
//                 row_indices.push_back(row);   // Store row index
//             }
//         }
//     }
//     col_pointers[ncols] = values.size(); // End of the last column

//     // Allocate memory for CSC arrays
    
//     params.data = (OSQPFloat*)malloc(values.size() * sizeof(OSQPFloat));
//     params.row_indices = (OSQPInt*)malloc(row_indices.size() * sizeof(OSQPInt));
//     params.col_pointers = (OSQPInt*)malloc(col_pointers.size() * sizeof(OSQPInt));

//     // Copy data from vectors to allocated arrays
//     std::cerr << "Copy data from vectors to allocated arrays: "<< std::endl;
//     std::copy(values.begin(), values.end(), params.data);
//     std::cerr << "Is it fault here 1: "<< std::endl;
//     std::copy(row_indices.begin(), row_indices.end(), params.row_indices);
//     std::cerr << "Is it fault here 2: "<< std::endl;
//     std::copy(col_pointers.begin(), col_pointers.end(), params.col_pointers);

//     params.nzeros = values.size(); // Number of non-zero elements
    
    

//     return params;
// }
  // Function to free the CSC params
  void free_csc_params(CSCMatrix_params& p) {
    // if (p.data.data()) free(p.data.data());
    // if (p.row_indices.data()) free(p.row_indices.data());
    // if (p.col_pointers.data()) free(p.col_pointers.data());
    // p.data.data() = nullptr; // Set to nullptr to prevent double-free
    // p.row_indices.data() = nullptr;
    // p.col_pointers.data() = nullptr;
}
  

 protected:
  
  double* u_;
  OSQPSettings *qp_setting_;
  ::OSQPSolver   *qp_solver_;
  OSQPInt      setup_flag_;
  OSQPInt      update_flag_;
  OSQPInt      solve_flag_;
  OSQPInt      num_var_;
  OSQPInt      num_constr_;

  OSQPCscMatrix *P_;// Pointer to the CSC matrix for the quadratic term in the cost function
  OSQPCscMatrix *A_; // Pointer to the CSC matrix for constraint

  OSQPFloat*     q_; // The linear term in the cost function
  OSQPFloat*     lower_; // Lower bound constraint
  OSQPFloat*     upper_; // Upper bound constraint

}; // end class OSQPSolver

} // end namespace labrob::qpsolvers
} // end namespace labrob

#endif // LABROB_HPIPM_QP_SOLVER_HPP_