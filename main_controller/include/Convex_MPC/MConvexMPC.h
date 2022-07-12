#ifndef _MCONVEX_MPC_
#define _MCONVEX_MPC_
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <OsqpEigen/OsqpEigen.h>
#include <stdio.h>
#include <sys/time.h>
#include <cpptypes.h>
//#define K_PRINT_EVERYTHING
using namespace Eigen;
using namespace OsqpEigen;

class MPC_SLOVER
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	int NumberOfVariables;
	int NumberOfConstraints;
	OsqpEigen::Solver solver;
	VectorXd lowerC;
	VectorXd upperC;
	VectorXd QPSolution;

	Matrix<double, Dynamic, 12> A_qp;
	Matrix<double, Dynamic, 1> G_qp;
	MatrixXd B_qp;

	Matrix<double, 12, 12> Bdt;
	Matrix<double, 12, 12> Adt;
	Vec12<double> Gdt;

	Matrix<double, 25, 25> ABc, expmm;
	Matrix<double, 3, 4> r_feet_world;
	Vec12<double> x_0;
	VectorXd X_d;
	VectorXd U_b;
	Matrix<double, 3, 3> R_yaw;
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

	MPC_SLOVER(uint32_t horizon_,double dtMPC_) : horizon(horizon_),dtMPC(dtMPC_)
	{
		if (horizon > 19)
		{
			throw std::runtime_error("horizon is too long!");
		}
		mass = 13;
		alpha = 1e-7;
		mu = 0.4;
		muinv = 1.f / mu;
		NumberOfVariables = 12 * horizon;
		NumberOfConstraints = 20 * horizon;
		L_weights << 20.0, 10.0, 1.0,
			0.0, 0.0, 420.0,
			0.05, 0.05, 0.05,
			30.0, 30.0, 10.0;
		I_body<<0.0158533,-3.66e-5, -6.11e-5, -3.66e-5 ,0.0377999 ,-2.75e-5 ,-6.11e-5 ,- 2.75e-5 ,0.0456542;
		initialize();
	}
	~MPC_SLOVER() {}

	void initialize();
	void solve_mpc(double phi,Vec12<double> &x_0_, Mat34<double> &r_feet_w_, int *contact_state, double *traj,Vec12<double>& force_);

private:
	// continuous time state space matrices.
	void ct_ss_mats();
	void c2qp();

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