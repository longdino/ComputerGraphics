#include <iostream>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "Cloth.h"
#include "Particle.h"
#include "Spring.h"
#include "MatrixStack.h"
#include "Program.h"
#include "GLSL.h"
#include "snh.h"
#include "Triangle.h"

using namespace std;
using namespace Eigen;

shared_ptr<Spring> createSpring(const shared_ptr<Particle> p0, const shared_ptr<Particle> p1, double E)
{
	auto s = make_shared<Spring>(p0, p1);
	s->E = E;
	Vector2d x0 = p0->x;
	Vector2d x1 = p1->x;
	Vector2d dx = x1 - x0;
	s->L = dx.norm();
	return s;
}

shared_ptr<Triangle> triangle(const shared_ptr<Particle> p0, const shared_ptr<Particle> p1, const shared_ptr<Particle> p2, double E) {
	auto t = make_shared<Triangle>(p0, p1, p2);
	t->E = E;

	return t;
}

Cloth::Cloth(int rows, int cols,
	const Vector2d& x00,
	const Vector2d& x01,
	const Vector2d& x10,
	const Vector2d& x11,
	double mass,
	double stiffness)
{
	assert(rows > 1);
	assert(cols > 1);
	assert(mass > 0.0);
	assert(stiffness > 0.0);

	this->rows = rows;
	this->cols = cols;

	// Create particles
	n = 0;
	double r = 0.02; // Used for collisions
	int nVerts = rows * cols;
	for (int i = 0; i < rows; ++i) {
		double u = i / (rows - 1.0);
		Vector2d x0 = (1 - u) * x00 + u * x10;
		Vector2d x1 = (1 - u) * x01 + u * x11;
		for (int j = 0; j < cols; ++j) {
			double v = j / (cols - 1.0);
			Vector2d x = (1 - v) * x0 + v * x1;
			auto p = make_shared<Particle>();
			particles.push_back(p);
			p->r = r;
			p->x = x;
			p->v << 0.0, 0.0;
			p->m = mass / (nVerts);
			// Pin two particles
			//if(i == 0 && (j == 0 || j == cols-1)) {
			//	p->fixed = true;
			//	p->i = -1;
			//}
//			else {
			p->fixed = false;
			p->i = n;
			n += 2;
			//			}
		}
	}

	//cout << particles.size() << endl;

	// Create Triangles
	vector<float> elem;
	for (int i = 0; i < cols - 1; i++) {
		for (int j = 0; j < rows - 1; j++) {
			int k0 = rows * j + i;
			int k1 = rows * j + i + 1;
			int k2 = rows * j + i + rows;

			int m0 = rows * j + i + 1;
			int m1 = rows * j + i + rows;
			int m2 = rows * j + i + rows + 1;

			triangles.push_back(triangle(particles[k0], particles[k1], particles[k2], stiffness));
			triangles.push_back(triangle(particles[m0], particles[m1], particles[m2], stiffness));

			elem.push_back(k0);
			elem.push_back(k1);
			elem.push_back(k2);
			elem.push_back(m0);
			elem.push_back(m1);
			elem.push_back(m2);
		}
	}

	// Build system matrices and vectors
	M.resize(n, n);
	K.resize(n, n);
	v.resize(n);
	f.resize(n);

	// Build vertex buffers
	posBuf.clear();
	//norBuf.clear();
	texBuf.clear();
	eleBuf.clear();
	posBuf.resize(nVerts * 3);
	//norBuf.resize(nVerts*3);
	updatePosNor();

	// Texture coordinates (don't change)
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			texBuf.push_back(i / (rows - 1.0));
			texBuf.push_back(j / (cols - 1.0));
		}
	}

	// Elements (don't change)
	for (int i = 0; i < elem.size(); ++i) {
		eleBuf.push_back(elem[i]);
	}

	//cout << posBuf.size() << endl;

}

Cloth::~Cloth()
{
}

void Cloth::tare()
{
	for (int k = 0; k < (int)particles.size(); ++k) {
		particles[k]->tare();
	}
}

void Cloth::reset()
{
	for (int k = 0; k < (int)particles.size(); ++k) {
		particles[k]->reset();
	}
	updatePosNor();
}

void Cloth::updatePosNor()
{
	// Position
	//for(int i = 0; i < rows; ++i) {
	//	for(int j = 0; j < cols; ++j) {
	//		int k = i*cols + j;
	//		Vector2d x = particles[k]->x;
	//		posBuf[3*k+0] = x(0);
	//		posBuf[3*k+1] = x(1);
	//	}
	//}

	for (int i = 0; i < particles.size(); ++i) {
		Vector2d x = particles[i]->x;
		posBuf[3 * i + 0] = x(0);
		posBuf[3 * i + 1] = x(1);
	}
}

void Cloth::step(double h, double mu, double lambda, const Vector2d& grav, const vector< shared_ptr<Particle> > spheres)
{
	M.setZero();
	K.setZero();
	v.setZero();
	f.setZero();

	//
	// IMPLEMENT ME!
	//

	// Sparse Matrix
	int res = M.rows();

	typedef Eigen::Triplet<double> T;
	vector<T> m_;
	vector<T> k_;

	Eigen::SparseMatrix<double> A_(res, res);
	Eigen::SparseMatrix<double> M_(res, res);
	Eigen::SparseMatrix<double> K_(res, res);

	// Explicit Integration
	// step 1: fill in M, v, and f
	int index = 0;
	for (int i = 0; i < particles.size(); ++i) {
		if (particles[i]->i != -1) {
			index = particles[i]->i;
			double m = particles[i]->m;
			v.segment(index, 2) = particles[i]->v;
			f.segment(index, 2) = m * grav;
			m_.push_back(T(index, index, m));
			m_.push_back(T(index + 1, index + 1, m));
		}
	}

	// Stable Neo-Hookean Simulation
	for (int i = 0; i < triangles.size(); ++i) {
		fs.setZero();
		Ks.setZero();


		int index0 = triangles[i]->p0->i;
		int index1 = triangles[i]->p1->i;
		int index2 = triangles[i]->p2->i;

		auto s = make_shared<Snh>();

		VectorXd x0 = triangles[i]->p0->x;
		VectorXd x1 = triangles[i]->p1->x;
		VectorXd x2 = triangles[i]->p2->x;

		VectorXd X0 = triangles[i]->p0->X;
		VectorXd X1 = triangles[i]->p1->X;
		VectorXd X2 = triangles[i]->p2->X;

		MatrixXd dX(2, 2);
		dX << (X1 - X0)(0), (X2 - X0)(0),
			(X1 - X0)(1), (X2 - X0)(1);

		double a = dX.determinant();
		MatrixXd dXinv = dX.inverse();

		s->snh(x0, x1, x2, dXinv, a, mu, lambda, fs, Ks);

		MatrixXd negKs = Ks;

		// add fs to f
		Vector2d f0 = fs.segment<2>(0);
		Vector2d f1 = fs.segment<2>(2);
		Vector2d f2 = fs.segment<2>(4);
		// add Ks to K sparse matrix
		Matrix2d K00 = Ks.block<2, 2>(0, 0);
		Matrix2d K01 = Ks.block<2, 2>(0, 2);
		Matrix2d K02 = Ks.block<2, 2>(0, 4);
		Matrix2d K10 = Ks.block<2, 2>(2, 0);
		Matrix2d K11 = Ks.block<2, 2>(2, 2);
		Matrix2d K12 = Ks.block<2, 2>(2, 4);
		Matrix2d K20 = Ks.block<2, 2>(4, 0);
		Matrix2d K21 = Ks.block<2, 2>(4, 2);
		Matrix2d K22 = Ks.block<2, 2>(4, 4);

		cout << fs << endl;
		cout << Ks << endl;

		if (index0 != -1) {
			f.segment(index0, 2) += f0;
			for (int j = 0; j < 2; j++) {
				for (int k = 0; k < 2; k++) {
					if (K00(j, k) != 0) {
						k_.push_back(T(index0 + j, index0 + k, K00(j, k)));
					}
				}
			}
		}
		if (index1 != -1) {
			f.segment(index1, 2) += f1;
			for (int j = 0; j < 2; j++) {
				for (int k = 0; k < 2; k++) {
					if (K11(j, k) != 0) {
						k_.push_back(T(index1 + j, index1 + k, K11(j, k)));
					}
				}
			}
		}
		if (index2 != -1) {
			f.segment(index2, 2) += f2;
			for (int j = 0; j < 2; j++) {
				for (int k = 0; k < 2; k++) {
					if (K22(j, k) != 0) {
						k_.push_back(T(index2 + j, index2 + k, K22(j, k)));
					}
				}
			}
		}
		if (index0 != -1 && index1 != -1) {
			for (int j = 0; j < 2; j++) {
				for (int k = 0; k < 2; k++) {
					if (K01(j, k) != 0) {
						k_.push_back(T(index0 + j, index1 + k, K01(j, k)));
					}
					if (K10(j, k) != 0) {
						k_.push_back(T(index1 + j, index0 + k, K10(j, k)));
					}
				}
			}
		}
		if (index0 != -1 && index2 != -1) {
			for (int j = 0; j < 2; j++) {
				for (int k = 0; k < 2; k++) {
					if (K02(j, k) != 0) {
						k_.push_back(T(index0 + j, index2 + k, K02(j, k)));
					}
					if (K20(j, k) != 0) {
						k_.push_back(T(index2 + j, index0 + k, K20(j, k)));
					}
				}
			}
		}
		if (index1 != -1 && index2 != -1) {
			for (int j = 0; j < 2; j++) {
				for (int k = 0; k < 2; k++) {
					if (K12(j, k) != 0) {
						k_.push_back(T(index1 + j, index2 + k, K12(j, k)));
					}
					if (K21(j, k) != 0) {
						k_.push_back(T(index2 + j, index1 + k, K21(j, k)));
					}
				}
			}
		}

		cout << "f: \n" << f << endl;
		cout << "K: \n" << K << endl;
	}

	// Sphere collision
	//double c = 1e2;
	//for (int i = 0; i < particles.size(); i++) {
	//	for (int j = 0; j < spheres.size(); j++) {
	//		Vector2d delPos = particles[i]->x - spheres[j]->x;
	//		double l = delPos.norm();
	//		double d = particles[i]->r + spheres[j]->r - l;

	//		Matrix2d I;
	//		I.setIdentity();

	//		if (d > 0) {
	//			index = particles[i]->i;
	//			Vector2d fc = c * d * delPos / l;
	//			Matrix2d Kc = c * d * I;

	//			Matrix2d negKc = - (Kc*Kc);

	//			f.segment(index, 2) += fc;
	//			for (int k = 0; k < Kc.rows(); k++) {
	//				for (int m = 0; m < Kc.cols(); m++) {
	//					if (negKc(k, m) != 0) {
	//						k_.push_back(T(index + k, index + m, Kc(k, m)));
	//					}
	//				}
	//			}
	//		}
	//	}
	//}

	// step 3: solve linear system Ax = b
	VectorXd b;
	VectorXd x;

	// solve using a sparse matrix
	M_.setFromTriplets(m_.begin(), m_.end());
	K_.setFromTriplets(k_.begin(), k_.end());

	A_ = M_ - pow(h, 2) * K;
	b = M_ * v + h * f;

	ConjugateGradient< SparseMatrix<double> > cg;
	cg.setMaxIterations(25);
	cg.setTolerance(1e-6);
	cg.compute(A_);

	x = cg.solveWithGuess(b, v);

	// step 4: extract velocites
	for (int i = 0; i < particles.size(); ++i) {
		if (particles[i]->i != -1) {
			index = particles[i]->i;
			particles[i]->v = x.segment(index, 2);
		}
	}

	// step 5: integrate positions with new velocities
	for (int i = 0; i < particles.size(); ++i) {
		if (particles[i]->i != -1) {
			particles[i]->x += h * particles[i]->v;
		}
	}
	// Update position and normal buffers
	updatePosNor();
}

void Cloth::init()
{
	glGenBuffers(1, &posBufID);
	glBindBuffer(GL_ARRAY_BUFFER, posBufID);
	glBufferData(GL_ARRAY_BUFFER, posBuf.size() * sizeof(float), &posBuf[0], GL_DYNAMIC_DRAW);

	glGenBuffers(1, &texBufID);
	glBindBuffer(GL_ARRAY_BUFFER, texBufID);
	glBufferData(GL_ARRAY_BUFFER, texBuf.size() * sizeof(float), &texBuf[0], GL_STATIC_DRAW);

	glGenBuffers(1, &eleBufID);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, eleBufID);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, eleBuf.size() * sizeof(unsigned int), &eleBuf[0], GL_STATIC_DRAW);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	assert(glGetError() == GL_NO_ERROR);
}

void Cloth::draw(shared_ptr<MatrixStack> MV, const shared_ptr<Program> p) const
{
	// Draw mesh
	//glUniform3fv(p->getUniform("kdFront"), 1, Vector3f(1.0, 0.0, 0.0).data());
	//glUniform3fv(p->getUniform("kdBack"),  1, Vector3f(1.0, 1.0, 0.0).data());
	MV->pushMatrix();
	glUniformMatrix4fv(p->getUniform("MV"), 1, GL_FALSE, glm::value_ptr(MV->topMatrix()));
	int h_pos = p->getAttribute("aPos");
	glEnableVertexAttribArray(h_pos);
	glBindBuffer(GL_ARRAY_BUFFER, posBufID);
	glBufferData(GL_ARRAY_BUFFER, posBuf.size() * sizeof(float), &posBuf[0], GL_STATIC_DRAW);
	glVertexAttribPointer(h_pos, 3, GL_FLOAT, GL_FALSE, 0, (const void*)0);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, eleBufID);
	//for(int i = 0; i < rows; ++i) {
	//	glDrawElements(GL_TRIANGLE_STRIP, 2*cols, GL_UNSIGNED_INT, (const void *)(2*cols*i*sizeof(unsigned int)));
	//}
	int nElements = (int)eleBuf.size();
	glDrawElements(GL_TRIANGLES, nElements, GL_UNSIGNED_INT, 0);

	glDisableVertexAttribArray(h_pos);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	MV->popMatrix();
}
