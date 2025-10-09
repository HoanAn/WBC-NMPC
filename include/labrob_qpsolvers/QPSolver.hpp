#ifndef LABROB_QP_SOLVER_HPP_
#define LABROB_QP_SOLVER_HPP_

#include <iostream>

namespace labrob {
namespace qpsolvers {

struct CSCMatrix_params {
    long long nrows;              // Number of rows
    long long ncols;              // Number of columns
    long long nzeros;             // Non-zero values
    std::vector<double> data;            // Vector of data
    std::vector<long long>   row_indices;     // Vector of row indices
    std::vector<long long>   col_pointers;    // Column pointers (start of each column in values array)
  };

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

  virtual void solve_CCS(
    const CSCMatrix_params& H,
    const Scalar* g,
    const CSCMatrix_params& A,  
    const Scalar* d_min,
    const Scalar* d_max) = 0;

  virtual const Scalar* get_solution() const = 0;

  const int num_variables_;
  const int num_equality_constraints_;
  const int num_inequality_constraints_;

  

}; // end class QPSolver


/* Convert a dense matrix to CSC format and create an OSQPCscMatrix 
H : a dense matrix
nrows : number of rows of H
ncols : number of columns of H
*/
inline CSCMatrix_params denseToCSC_param(const double* H, int nrows, int ncols, double tol = 1e-12) {
    CSCMatrix_params params;
    params.nrows = nrows;
    params.ncols = ncols;

    std::vector<double> values;
    std::vector<long long> row_indices;
    std::vector<long long> col_pointers(ncols + 1, 0);

    // Quick symmetry check
    bool isSym = true;
    if (nrows == ncols) {
        for (int j = 0; j < ncols && isSym; j++) {
            for (int i = j + 1; i < nrows; i++) {
                double a = H[i + j * nrows];  // H(i,j) (column-major)
                double b = H[j + i * nrows];  // H(j,i)
                if (std::abs(a - b) > tol) {
                    isSym = false;
                    break;
                }
            }
        }
    } else {
        isSym = false;
    }

    // Build CSC format
    for (int col = 0; col < ncols; ++col) {
        col_pointers[col] = static_cast<long long>(values.size()); // start of column

        if (isSym) {
            for (int row = 0; row <= col && row < nrows; ++row) { // upper triangle only
                double value = H[row + col * nrows]; // column-major access
                if (std::abs(value) > tol) {
                    values.push_back(value);
                    row_indices.push_back(row);
                }
            }
        } else {
            for (int row = 0; row < nrows; ++row) {
                double value = H[row + col * nrows]; // column-major access
                if (std::abs(value) > tol) {
                    values.push_back(value);
                    row_indices.push_back(row);
                }
            }
        }
    }
    col_pointers[ncols] = static_cast<long long>(values.size());

    // Move into struct
    params.data = std::move(values);
    params.row_indices = std::move(row_indices);
    params.col_pointers = std::move(col_pointers);
    params.nzeros = static_cast<long long>(params.data.size());

    return params;
}

inline Eigen::VectorXd cscToDenseVector(const CSCMatrix_params& csc) {
    Eigen::VectorXd dense(csc.nrows * csc.ncols);
    dense.setZero();
    //std::cout << "Converting CSC to dense matrix (" <<csc.nrows <<"," <<csc.ncols<<")" << std::endl;
    for (int col = 0; col < csc.ncols; col++) {
        for (long long idx = csc.col_pointers[col]; idx < csc.col_pointers[col+1]; idx++) {
            int row = csc.row_indices[idx];
            double val = csc.data[idx];
            dense(row + col * csc.nrows) = val;  // column-major indexing
            //std::cout << "Setting dense(" << row << "," << col << ") = " << dense(row + col * csc.nrows) << std::endl;
        }
    }

    return dense;
  }
} // end namespace labrob::qpsolvers
} // end namespace labrob

#endif // LABROB_QP_SOLVER_HPP_