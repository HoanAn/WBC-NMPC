#pragma once

#include <vector>

#include "osqp/osqp.h"
#include "eigen3/Eigen/Sparse"
#include "eigen3/Eigen/Dense"

using OSQPVector = Eigen::VectorX<OSQPFloat>;
using OSQPMatrix = Eigen::MatrixX<OSQPFloat>;
using OSQPSparseMatrix = Eigen::SparseMatrix<OSQPFloat>;
using OSQPTriplet = Eigen::Triplet<OSQPFloat>;

struct QPProblem
{
    std::size_t n, m;
    OSQPSparseMatrix H, Ac;
    OSQPVector g, lc, uc;

    QPProblem(const std::size_t n, const std::size_t m)
        : n(n), m(m)
    {
        H = OSQPSparseMatrix(n, n);
        g = OSQPVector::Zero(n);
        Ac = OSQPSparseMatrix(m, n);
        lc = OSQPVector::Zero(m);
        uc = OSQPVector::Zero(m);
    }
};

struct QPSolution
{
    OSQPInt exit_flag;
    std::string exit_message;
    double run_time, setup_time, solve_time; // in seconds
    OSQPVector zstar;

    QPSolution(const OSQPSolver *solver, const QPProblem &qp)
    {
        exit_flag = solver->info->status_val;
        exit_message = solver->info->status;
        run_time = solver->info->run_time;
        setup_time = solver->info->setup_time;
        solve_time = solver->info->solve_time;

        if (exit_flag == OSQP_SOLVED)
        {
            zstar = Eigen::Map<OSQPVector>(solver->solution->x, qp.n);
        }
    }
};

static OSQPCscMatrix convertEigenSparseToCSC(const OSQPSparseMatrix &matrix)
{
    OSQPCscMatrix M;
    M.m = matrix.rows();
    M.n = matrix.cols();
    M.nz = -1;
    M.nzmax = matrix.nonZeros();
    M.x = new OSQPFloat[M.nzmax];
    M.i = new OSQPInt[M.nzmax];
    M.p = new OSQPInt[M.n + 1];

    int k = 0;
    M.p[0] = 0;
    for (int j = 0; j < matrix.outerSize(); ++j)
    {
        for (OSQPSparseMatrix::InnerIterator it(matrix, j); it; ++it)
        {
            M.x[k] = it.value();
            M.i[k] = it.row();
            ++k;
        }
        M.p[j + 1] = k;
    }

    return M;
}

static void cleanupCSC(OSQPCscMatrix &M)
{
    delete[] M.x;
    delete[] M.i;
    delete[] M.p;
}

static QPSolution solveOSQP(const QPProblem &qp, OSQPSettings *settings)
{
    OSQPCscMatrix P = convertEigenSparseToCSC(qp.H);
    OSQPCscMatrix A = convertEigenSparseToCSC(qp.Ac);

    OSQPSolver *solver;
    if (osqp_setup(&solver, &P, qp.g.data(), &A, qp.lc.data(), qp.uc.data(), qp.m, qp.n, settings) == 0)
    {
        osqp_solve(solver);
    }
    QPSolution solution(solver, qp);
    
    osqp_cleanup(solver);
    cleanupCSC(P);
    cleanupCSC(A);

    return solution;
}

struct MPCProblem
{
    std::size_t N;
    OSQPVector x0;
    std::vector<OSQPVector> xref, lbx, ubx, lbu, ubu;
    std::vector<OSQPMatrix> Q, R, A, B, C, D;

    MPCProblem(const std::size_t N)
        : N(N)
    {
        xref.resize(N);
        Q.resize(N);
        R.resize(N - 1);
        A.resize(N - 1);
        B.resize(N - 1);
        C.resize(N);
        lbx.resize(N);
        ubx.resize(N);
        D.resize(N - 1);
        lbu.resize(N - 1);
        ubu.resize(N - 1);
    }
};

static void addNonZeroTriplets(std::vector<OSQPTriplet> &triplets, const OSQPMatrix &matrix,
                               const Eigen::Index &roff, const Eigen::Index &coff)
{
    for (Eigen::Index i = 0; i < matrix.rows(); ++i)
    {
        for (Eigen::Index j = 0; j < matrix.cols(); ++j)
        {
            const OSQPFloat val = matrix(i, j);
            if (val != 0.0)
            {
                triplets.push_back(OSQPTriplet(roff + i, coff + j, val));
            }
        }
    }
}

static QPProblem getQPProblem(const MPCProblem &mpc)
{
    assert(mpc.N > 1);

    const std::size_t num_states = mpc.Q[0].cols();
    const std::size_t Nx = num_states * mpc.N;

    std::size_t Nu = 0;
    std::size_t Nd = 0;
    for (std::size_t i = 0; i < mpc.N - 1; ++i)
    {
        Nu += mpc.B[i].cols();
        Nd += mpc.D[i].rows();
    }

    std::size_t Nc = 0;
    for (std::size_t i = 0; i < mpc.N; ++i)
    {
        Nc += mpc.C[i].rows();
    }

    const std::size_t n = Nx + Nu;
    const std::size_t m = Nx + Nc + Nd;
    QPProblem qp(n, m);

    assert(std::size_t(mpc.x0.rows()) == num_states);
    qp.lc.segment(0, num_states) = -mpc.x0;
    qp.uc.segment(0, num_states) = -mpc.x0;

    std::size_t nu_offset = 0, nc_offset = 0, nd_offset = 0;
    std::vector<OSQPTriplet> H_triplets;
    std::vector<OSQPTriplet> Ac_triplets;
    for (std::size_t i = 0; i < mpc.N; ++i)
    {
        const std::size_t nx_offset = num_states * i;
        const std::size_t num_controls = mpc.R[i].cols();
        const std::size_t num_constraints_x = mpc.C[i].rows();

        // Hessian
        assert(std::size_t(mpc.Q[i].rows()) == num_states && std::size_t(mpc.Q[i].cols()) == num_states);
        addNonZeroTriplets(H_triplets, mpc.Q[i], nx_offset, nx_offset);

        // Gradient
        assert(std::size_t(mpc.xref[i].rows()) == num_states);
        qp.g.segment(nx_offset, num_states) = -mpc.Q[i] * mpc.xref[i];

        // Linear Constraints
        assert(std::size_t(mpc.C[i].cols()) == num_states);
        addNonZeroTriplets(Ac_triplets, mpc.C[i], Nx + nc_offset, nx_offset);

        // Control bounds
        assert(std::size_t(mpc.lbx[i].rows()) == num_constraints_x);
        qp.lc.segment(Nx + nc_offset, mpc.C[i].rows()) = mpc.lbx[i];

        assert(std::size_t(mpc.ubx[i].rows()) == num_constraints_x);
        qp.uc.segment(Nx + nc_offset, mpc.C[i].rows()) = mpc.ubx[i];

        if (i < mpc.N - 1)
        {
            const std::size_t num_constraints_u = mpc.D[i].rows();

            // Hessian
            assert(std::size_t(mpc.R[i].rows()) == num_controls && std::size_t(mpc.R[i].cols()) == num_controls);
            addNonZeroTriplets(H_triplets, mpc.R[i], Nx + nu_offset, Nx + nu_offset);

            // Linear Constraints
            assert(std::size_t(mpc.A[i].rows()) == num_states && std::size_t(mpc.A[i].cols()) == num_states);
            addNonZeroTriplets(Ac_triplets, mpc.A[i], nx_offset + num_states, nx_offset);

            assert(std::size_t(mpc.B[i].rows()) == num_states);
            addNonZeroTriplets(Ac_triplets, mpc.B[i], nx_offset + num_states, Nx + nu_offset);

            assert(std::size_t(mpc.D[i].cols()) == num_controls);
            addNonZeroTriplets(Ac_triplets, mpc.D[i], Nx + Nc + nd_offset, Nx + nu_offset);

            // Control bounds
            assert(std::size_t(mpc.lbu[i].rows()) == num_constraints_u);
            qp.lc.segment(Nx + Nc + nd_offset, mpc.D[i].rows()) = mpc.lbu[i];

            assert(std::size_t(mpc.ubu[i].rows()) == num_constraints_u);
            qp.uc.segment(Nx + Nc + nd_offset, mpc.D[i].rows()) = mpc.ubu[i];

            nu_offset += num_controls;
            nd_offset += num_constraints_u;
        }

        nc_offset += num_constraints_x;
    }

    for (std::size_t i = 0; i < Nx; ++i)
    {
        Ac_triplets.push_back(OSQPTriplet(i, i, -1.0));
    }

    qp.H.setFromTriplets(H_triplets.begin(), H_triplets.end());
    qp.Ac.setFromTriplets(Ac_triplets.begin(), Ac_triplets.end());

    return qp;
}

struct MPCSolution : public QPSolution
{
    std::vector<OSQPVector> xstar;
    std::vector<OSQPVector> ustar;

    MPCSolution(const QPSolution &qp_sol, const MPCProblem &mpc)
        : QPSolution(qp_sol)
    {
        if (exit_flag != OSQP_SOLVED)
        {
            return;
        }

        xstar.resize(mpc.N);
        for (std::size_t i = 0; i < mpc.N; ++i)
        {
            const auto num_states = mpc.Q[i].cols();
            xstar[i] = qp_sol.zstar.segment(i * num_states, num_states);
        }

        ustar.resize(mpc.N - 1);
        Eigen::Index nu_offset = 0;
        for (std::size_t i = 0; i < mpc.N - 1; ++i)
        {
            const auto num_states = mpc.Q[i].cols();
            const auto num_controls = mpc.R[i].cols();
            ustar[i] = qp_sol.zstar.segment(mpc.N * num_states + nu_offset, num_controls);
            nu_offset += num_controls;
        }
    }
};

static MPCSolution solveMPC(const MPCProblem &mpc, OSQPSettings *settings)
{
    QPProblem qp = getQPProblem(mpc);

    QPSolution qp_sol = solveOSQP(qp, settings);

    return MPCSolution(qp_sol, mpc);
}