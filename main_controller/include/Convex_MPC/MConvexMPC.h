#ifndef _MCONVEX_MPC_
#define _MCONVEX_MPC_
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <OsqpEigen/OsqpEigen.h>
#include <stdio.h>
#include <sys/time.h>

//#define K_PRINT_EVERYTHING
using namespace Eigen;
using namespace OsqpEigen;

class MPC_SLOVER
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	int NumberOfVariables;
	int NumberOfConstraints;
	OsqpEigen::Solver &solver;
	VectorXd lowerC;
	VectorXd upperC;
	VectorXd QPSolution;

	Matrix<double, Dynamic, 12> A_qp;
	Matrix<double, Dynamic, 1> G_qp;
	MatrixXd B_qp;

	Matrix<double, 12, 12> Bdt;
	Matrix<double, 12, 12> Adt;
	Vector<double, 12> Gdt;
	Matrix<double, 25, 25> ABc, expmm;
	Matrix<double, 3, 4> r_feet;
	VectorXd X_d;
	VectorXd U_b;
	Matrix<double, 3, 3> R_yaw;
	Matrix<double, 12, 1> x_0;
	Matrix<double, 3, 3> I_world, I_body;
	Matrix<double, 12, 12> Act;
	Matrix<double, 12, 12> Bct;

	Matrix<double, Dynamic, Dynamic> C;
	SparseMatrix<double> C_sparse;
	SparseMatrix<double> H;
	SparseMatrix<double> L;
	SparseMatrix<double> K;
	Matrix<double, 12, 1> L_weights;
	VectorXd g;
	double mu;
	double muinv;
	double mass;
	double dtMPC;
	double alpha;
	uint32_t horizon;

	MPC_SLOVER(uint32_t horizon_, OsqpEigen::Solver &solver_) : horizon(horizon_), solver(solver_)
	{
		if (horizon > 19)
		{
			throw std::runtime_error("horizon is too long!");
		}
		mass = 10;
		alpha = 1e-7;
		mu = 0.4;
		muinv = 1.f / mu;
		NumberOfVariables = 12 * horizon;
		NumberOfConstraints = 20 * horizon;
		L_weights << 20.0, 10.0, 1.0,
			0.0, 0.0, 420.0,
			0.05, 0.05, 0.05,
			30.0, 30.0, 10.0;
		I_body.setIdentity();
		initialize();
	}
	~MPC_SLOVER() {}

	void initialize();
	void solve_mpc(int *contact_state, double *traj);

private:
	// continuous time state space matrices.
	void ct_ss_mats();
	void c2qp();

	double sq(double a)
	{
		return a * a;
	}

	void quat_to_rpy(Quaterniond q, Matrix<double, 3, 1> &rpy)
	{
		double as = std::min(-2. * (q.x() * q.z() - q.w() * q.y()), .99999);
		rpy(0) = atan2(2.0 * (q.x() * q.y() + q.w() * q.z()), sq(q.w()) + sq(q.x()) - sq(q.y()) - sq(q.z()));
		rpy(1) = asin(as);
		rpy(2) = atan2(2.0 * (q.y() * q.z() + q.w() * q.x()), sq(q.w()) - sq(q.x()) - sq(q.y()) + sq(q.z()));
	}

	Matrix<double, 3, 3> cross_mat(Matrix<double, 3, 3> I_inv, Matrix<double, 3, 1> r)
	{
		Matrix<double, 3, 3> cm;
		cm << 0.f, -r(2), r(1),
			r(2), 0.f, -r(0),
			-r(1), r(0), 0.f;
		return I_inv * cm;
	}
};

#endif