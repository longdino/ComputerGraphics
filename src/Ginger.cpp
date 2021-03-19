#include <iostream>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#define CUTE_C2_IMPLEMENTATION
#include "cute_c2.h"

#include "Ginger.h"
#include "Particle.h"
#include "Triangle.h"
#include "MatrixStack.h"
#include "Program.h"
#include "GLSL.h"
#include "snh.h"
#include "Shape.h"
#include "Plane.h"

using namespace std;
using namespace Eigen; 

// cute_c2 variables
c2Circle circ_head;
c2Capsule cap_rArm;
c2Capsule cap_lArm;
c2Capsule cap_rLeg;
c2Capsule cap_lLeg;
c2Capsule cap_torso;

shared_ptr<Triangle> createTriangle(const shared_ptr<Particle> p0, const shared_ptr<Particle> p1, const shared_ptr<Particle> p2, double E) {
	auto t = make_shared<Triangle>(p0, p1, p2);
	t->E = E;

	return t;
}

Ginger::Ginger(const shared_ptr<Shape> g, const vector<float> pos, const vector<float> tex, const vector<float> elem, double mass, double stiffness, double radius) {
	assert(mass > 0.0);
	assert(stiffness > 0.0);

	this->size = pos.size();

	// Create particles
	n = 0;
	double r = radius;
	int nVerts = pos.size() / 3;
	bool duplicate = false;
	for (int i = 0; i < nVerts; ++i) {
		Vector2d x;
		x << pos[3 * i + 0], pos[3 * i + 1];
		for (int j = 0; j < particles.size(); j++) {
			if (particles[j]->x(0) == x(0) && particles[j]->x(1) == x(1)) {

				duplicate = true;

			}
		}
		if (!duplicate) {
			auto p = make_shared<Particle>();
			particles.push_back(p);
			p->r = r;
			p->x = x;
			p->v << 0.0, 0.0;
			p->m = mass / (nVerts);
			//if (p->x(1) < 0.1 && p->x(0) < 0.3) {		// can be changed based on which particles you want to fix
			//	p->fixed = true;
			//	p->i = -1;
			//}
			//else {
			p->fixed = false;
			p->i = n;
			n += 2;
			//}
		}
		duplicate = false;
	}

	// Add triangles
	for (int i = 0; i < elem.size(); i += 3) {
		int j0 = elem[i];
		int j1 = elem[i + 1];
		int j2 = elem[i + 2];

		triangles.push_back(createTriangle(particles[j0], particles[j1], particles[j2], stiffness));
	}

	// Create c2Circle and c2Capsules on gingerman's head, torso, arms, and legs
	// head
	circ_head.p = c2V(0.0, 0.75);
	circ_head.r = 0.5;

	// torso
	cap_torso.r = 0.3;

	// right arm and left arm
	cap_rArm.r = 0.3;
	cap_rArm.a = c2V(0.3, 0.4);
	cap_rArm.b = c2V(0.7, 0.5);

	cap_lArm.r = 0.3;
	cap_lArm.a = c2V(-0.7, 0.5);
	cap_lArm.b = c2V(-0.3, 0.4);

	// right leg and left leg
	cap_rLeg.r = 0.3;
	cap_rLeg.a = c2V(0.3, -0.5);
	cap_rLeg.b = c2V(0.5, -1.0);

	cap_lLeg.r = 0.3;
	cap_lLeg.a = c2V(-0.5, -1.0);
	cap_lLeg.b = c2V(-0.3, -0.5);

	cout << cap_rLeg.r << endl;
	cout << "Collision: head-rightArm: " << c2CircletoCapsule(circ_head, cap_rArm) << endl;

	// build system matrices and vectors
	M.resize(n, n);
	K.resize(n, n);
	v.resize(n);
	f.resize(n);
	// Build vertex buffers
	posBuf.clear();
	texBuf.clear();
	eleBuf.clear();
	posBuf.resize(particles.size() * 3);
	updatePos();

	// Texture coordinates
	for (int i = 0; i < tex.size(); ++i) {
		texBuf.push_back(tex[i]);
	}

	// Elements
	for (int i = 0; i < elem.size(); ++i) {
		eleBuf.push_back(elem[i]);
	}
}

Ginger::~Ginger() {

}

void Ginger::tare() {
	for (int k = 0; k < (int)particles.size(); ++k) {
		particles[k]->tare();
	}
}

void Ginger::reset() {
	for (int k = 0; k < (int)particles.size(); ++k) {
		particles[k]->reset();
	}
	updatePos();
}

void Ginger::updatePos() {
	for (int i = 0; i < particles.size(); ++i) {
		Vector2d x = particles[i]->x;
		posBuf[3 * i + 0] = x(0);
		posBuf[3 * i + 1] = x(1);
	}
}

void Ginger::step(double h, double mu, double lambda, const Vector2d& grav, const shared_ptr<Plane> floor) {
	M.setZero();
	K.setZero();
	v.setZero();
	f.setZero();

	// Sparse Matrix
	int res = M.rows();

	typedef Eigen::Triplet<double> T;
	vector<T> m_;
	vector<T> k_;

	Eigen::SparseMatrix<double> A_(res, res);
	Eigen::SparseMatrix<double> M_(res, res);
	Eigen::SparseMatrix<double> K_(res, res);

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

	// Stable Neo-Hookean Computation
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
		//
		double a = dX.determinant();
		MatrixXd dXinv = dX.inverse();

		s->snh(x0, x1, x2, dXinv, a, mu, lambda, fs, Ks);

		// add fs to f
		//f.segment(index0, 6) += fs;
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
			//f.segment(index0, 2) += f0;
			//f.segment(index1, 2) += f1;
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
			//f.segment(index0, 2) += f0;
			//f.segment(index2, 2) += f2;
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
			//f.segment(index1, 2) += f1;
			//f.segment(index2, 2) += f2;
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

	}

	// collision force
	//fc = [fcx + fcy]'
	double c = 1e2;

	//for (int i = 0; i < particles.size(); i++) {
	//	for (int j = 0; j < floor->particles.size(); j++) {
	//		Vector2d delPos = particles[i]->x - floor->particles[j]->x;
	//		double l = delPos.norm();
	//		double d = particles[i]->r + floor->particles[j]->r - l;

	//		Matrix2d I;
	//		I.setIdentity();

	//		if (d > 0) {
	//			//cout << "collision distance: " << d << endl;
	//			index = particles[i]->i;
	//			Vector2d fc = c * d * delPos / l;
	//			Matrix2d Kc = c * d * I;

	//			f.segment(index, 2) += fc;
	//			for (int k = 0; k < Kc.rows(); k++) {
	//				for (int m = 0; m < Kc.cols(); m++) {
	//					if (Kc(k, m) != 0) {
	//						k_.push_back(T(index + k, index + m, Kc(k, m)));
	//					}
	//				}
	//			}
	//		}
	//	}
	//}

	for (int i = 0; i < particles.size(); i++) {

		Vector2d flrPos(particles[i]->x[0], floor->ymin);
		Vector2d delPos = particles[i]->x - flrPos;
		double d = delPos.norm();
		double dist = delPos[1];

		//cout << "floor: " << flrPos << endl;
		//cout << "delPos: " << delPos << endl;
		//cout << "d: " << d << endl;

		//cout << "Collision: head-rightArm: " << c2CircletoCapsule(circ_head, cap_rArm) << endl;

		Matrix2d I;
		I.setIdentity();

		if (dist < 0) {
			//cout << "under floor" << endl;
			index = particles[i]->i;
			Vector2d fc = c * dist * delPos / d;
			Matrix2d Kc = c * dist * I;

			f.segment(index, 2) += fc;
			for (int k = 0; i < Kc.rows(); k++) {
				for (int m = 0; m < Kc.cols(); m++) {
					if (Kc(k, m) != 0) {
						k_.push_back(T(index + k, index + m, Kc(k, m)));
					}
				}
			}
		}
	}

	// solve using a sparse matrix
	VectorXd b;
	VectorXd x;

	M_.setFromTriplets(m_.begin(), m_.end());
	K_.setFromTriplets(k_.begin(), k_.end());

	A_ = M_ - pow(h, 2) * K_;
	b = M_ * v + h * f;

	ConjugateGradient< SparseMatrix<double> > cg;
	cg.setMaxIterations(25);
	cg.setTolerance(1e-6);
	cg.compute(A_);

	x = cg.solveWithGuess(b, v);

	// extract velocites
	for (int i = 0; i < particles.size(); ++i) {
		if (particles[i]->i != -1) {
			index = particles[i]->i;
			particles[i]->v = x.segment(index, 2);
		}
	}

	// integrate positions with new velocities
	for (int i = 0; i < particles.size(); ++i) {
		if (particles[i]->i != -1) {
			particles[i]->x += h * particles[i]->v;
		}
	}

	// Update position buffers
	updatePos();

}

void Ginger::init() {
	// Send the position array to the GPU
	glGenBuffers(1, &posBufID);
	glBindBuffer(GL_ARRAY_BUFFER, posBufID);
	glBufferData(GL_ARRAY_BUFFER, posBuf.size() * sizeof(float), &posBuf[0], GL_DYNAMIC_DRAW);

	if (!texBuf.empty()) {
		glGenBuffers(1, &texBufID);
		glBindBuffer(GL_ARRAY_BUFFER, texBufID);
		glBufferData(GL_ARRAY_BUFFER, texBuf.size() * sizeof(float), &texBuf[0], GL_STATIC_DRAW);
	}

	if (!eleBuf.empty()) {
		glGenBuffers(1, &eleBufID);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, eleBufID);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, eleBuf.size() * sizeof(unsigned int), &eleBuf[0], GL_STATIC_DRAW);
	}

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	assert(glGetError() == GL_NO_ERROR);
}

void Ginger::draw(shared_ptr<MatrixStack> MV, const shared_ptr<Program> p) const {
	// Draw mesh
	MV->pushMatrix();
	glUniformMatrix4fv(p->getUniform("MV"), 1, GL_FALSE, glm::value_ptr(MV->topMatrix()));
	int h_pos = p->getAttribute("aPos");
	glEnableVertexAttribArray(h_pos);
	glBindBuffer(GL_ARRAY_BUFFER, posBufID);
	glBufferData(GL_ARRAY_BUFFER, posBuf.size() * sizeof(float), &posBuf[0], GL_STATIC_DRAW);
	glVertexAttribPointer(h_pos, 3, GL_FLOAT, GL_FALSE, 0, (const void*)0);

	// Bind texcoords buffer
	int h_tex = -9999;
	if (!texBuf.empty()) {
		int h_tex = p->getAttribute("aTex");
		glEnableVertexAttribArray(h_tex);
		glBindBuffer(GL_ARRAY_BUFFER, texBufID);
		glVertexAttribPointer(h_tex, 2, GL_FLOAT, GL_FALSE, 0, (const void*)0);
	}
	// Draw
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, eleBufID);
	int nElements = (int)eleBuf.size();
	glDrawElements(GL_TRIANGLES, nElements, GL_UNSIGNED_INT, 0);

	glDisableVertexAttribArray(h_pos);
	if(h_tex != -9999) glDisableVertexAttribArray(h_tex);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	MV->popMatrix();
}