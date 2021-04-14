#pragma once
#ifndef _SNH_H
#define _SNH_H

#include <vector>
#include <memory>

#define EIGEN_DONT_ALIGN_STATICALLY
#include <Eigen/Dense>
#include <Eigen/Sparse>

class Snh {
public:
	Snh();
	virtual ~Snh();
	void snh(Eigen::VectorXd x0, Eigen::VectorXd x1, Eigen::VectorXd x2, Eigen::MatrixXd dXInv, double A, double mu, double lambda, Eigen::VectorXd& f, Eigen::MatrixXd& K);
	Eigen::VectorXd vec(Eigen::MatrixXd A);
	Eigen::MatrixXd DFDx(Eigen::MatrixXd DmInv);

private:
	double nu = 0.45;
	double young = 1e3;
};

#endif