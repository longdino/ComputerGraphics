#include <iostream>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "snh.h"
#include "MatrixStack.h"
#include "Program.h"
#include "GLSL.h"

using namespace std;
using namespace Eigen;

/*
* This code is converted from snh2.m
* Stable Neo-Hookean Flesh Simulation
* Smith et al. 2018
*/


Snh::Snh() {

}

Snh::~Snh() {

}

void Snh::snh(VectorXd x0, VectorXd x1, VectorXd x2, MatrixXd dXInv, double A, double mu, double lambda, VectorXd& f, MatrixXd& K) {
	//cout << "SNH" << endl;

	MatrixXd dx(2,2);
	dx << (x1 - x0)(0), (x2 - x0)(0),
		(x1 - x0)(1), (x2 - x0)(1);

	MatrixXd F = dx * dXInv;
	double J = F.determinant();
	MatrixXd C = F.transpose() * F;
	double IC = C.trace();

	// Energy Density
	double a = 1.0 + mu / lambda;
	double Ja = J - a;
	double Psi = (mu / 2.0) * (IC - 2.0) + (lambda / 2.0) * (Ja * Ja);

	// Energy
	double V = A * Psi;

	// PKI
	MatrixXd T(2, 2);
	T << 0, 1,
		-1, 0;
	MatrixXd dJdF = T * F * T.transpose();
	MatrixXd PKI = mu * F + lambda * Ja * dJdF;

	// Force
	MatrixXd changeOfBasis = DFDx(dXInv);
	f = -A * changeOfBasis.transpose() * vec(PKI);

	// Stiffness
	VectorXd g = vec(dJdF);
	MatrixXd DJ2(4, 4);
	DJ2 << 0, 0, 0, 1,
		0, 0, -1, 0,
		0, -1, 0, 0,
		1, 0, 0, 0;
	Matrix4d I = Matrix4d::Identity();
	MatrixXd H = mu * I + lambda * ((g * g.transpose()) + Ja * DJ2);
	K = -A * changeOfBasis.transpose() * H * changeOfBasis;
	
	//return V;
}

VectorXd Snh::vec(MatrixXd A) {
	int row = A.rows();
	int col = A.cols();

	Map<VectorXd> x(A.data(), row * col, 1);

	return x;
}

MatrixXd Snh::DFDx(MatrixXd DmInv) {
	vector<MatrixXd> matrices;
	vector<VectorXd> vectors;
	MatrixXd d1(2, 2);
	MatrixXd d2(2, 2);
	MatrixXd d3(2, 2);
	MatrixXd d4(2, 2);
	MatrixXd d5(2, 2);
	MatrixXd d6(2, 2);

	d1 << -1, -1,
		0, 0;
	matrices.push_back(d1);

	d2 << 0, 0,
		-1, -1;
	matrices.push_back(d2);

	d3 << 1, 0,
		0, 0;
	matrices.push_back(d3);

	d4 << 0, 0,
		1, 0;
	matrices.push_back(d4);

	d5 << 0, 1,
		0, 0;
	matrices.push_back(d5);

	d6 << 0, 0,
		0, 1;
	matrices.push_back(d6);

	MatrixXd final(4, 6);
	
	for (int i = 0; i < matrices.size(); i++) {
		VectorXd x = vec(matrices[i] * DmInv);
		vectors.push_back(x);
	}

	for (int i = 0; i < final.cols(); i++) {
		for (int j = 0; j < final.rows(); j++) {
			final(j, i) = vectors[i](j);
		}
	}

	return final;
}