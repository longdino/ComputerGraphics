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
c2Poly head;
c2Poly right_arm;
c2Poly left_arm;
c2Poly right_leg;
c2Poly left_leg;
c2Poly torso;
vector<vector<int>> body;
vector<int> cuteIndex;

// debugging
c2Poly tri1;
c2Poly tri2;
vector<vector<int>> debg;
vector<int> debgIndex;
c2Manifold m;

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

	//// debugging
	//for (int i = 0; i < 2; i++) {
	//	debg.push_back(vector<int>());
	//}

	//tri1.count = 3;
	//tri1.verts[0] = c2V(particles[9]->x[0], particles[9]->x[1]);
	//tri1.verts[1] = c2V(particles[14]->x[0], particles[14]->x[1]);
	//tri1.verts[2] = c2V(particles[10]->x[0], particles[10]->x[1]);
	//c2Norms(tri1.verts, tri1.norms, tri1.count);

	//debg[0].push_back(particles[9]->i);
	//debg[0].push_back(particles[14]->i);
	//debg[0].push_back(particles[10]->i);

	//tri2.count = 3;
	//tri2.verts[0] = c2V(particles[12]->x[0], particles[12]->x[1]);
	//tri2.verts[1] = c2V(particles[13]->x[0], particles[13]->x[1]);
	//tri2.verts[2] = c2V(particles[11]->x[0], particles[11]->x[1]);

	//c2Norms(tri2.verts, tri2.norms, tri2.count);

	//debg[1].push_back(particles[12]->i);
	//debg[1].push_back(particles[13]->i);
	//debg[1].push_back(particles[11]->i);

	//if (c2PolytoPoly(&tri1, nullptr, &tri2, nullptr)) {
	//	cout << "collision" << endl;
	//}

	//c2PolytoPolyManifold(&tri1, nullptr, &tri2, nullptr, &m);
	//cout << "m.count: " << m.count << endl;

	//for (int i = 0; i < m.count; i++) {
	//	cout << "collision point: " << m.contact_points[i].x << ", " << m.contact_points[i].y << endl;
	//	cout << "depth: " << m.depths[i] << endl;
	//	cout << "normal: " << m.n.x << ", " << m.n.y << endl;
	//}

	//for (int i = 0; i < tri1.count; i++) {
	//	cout << "(" << tri1.verts[i].x << ", " << tri1.verts[i].y << ")" << endl;
	//}
	//for (int i = 0; i < tri2.count; i++) {
	//	cout << "(" << tri2.verts[i].x << ", " << tri2.verts[i].y << ")" << endl;
	//}

	// Create c2Circle and c2Capsules on gingerman's head, torso, arms, and legs
	for (int i = 0; i < 6; i++) {
		body.push_back(vector<int>());
	}

	// head
	head.verts[0] = c2V(particles[3]->x[0], particles[3]->x[1]); //3
	head.verts[1] = c2V(particles[8]->x[0], particles[8]->x[1]); //8
	head.verts[2] = c2V(particles[10]->x[0], particles[10]->x[1]); //10
	head.verts[3] = c2V(particles[13]->x[0], particles[13]->x[1]); //13
	head.verts[4] = c2V(particles[12]->x[0], particles[12]->x[1]); //12
	head.verts[5] = c2V(particles[39]->x[0], particles[39]->x[1]); //39
	head.verts[6] = c2V(particles[41]->x[0], particles[41]->x[1]); //41
	head.verts[7] = c2V(particles[1]->x[0], particles[1]->x[1]); //1
	head.count = 8;
	c2Norms(head.verts, head.norms, head.count);

	body[0].push_back(particles[3]->i);
	body[0].push_back(particles[8]->i);
	body[0].push_back(particles[10]->i);
	body[0].push_back(particles[13]->i);
	body[0].push_back(particles[12]->i);
	body[0].push_back(particles[39]->i);
	body[0].push_back(particles[41]->i);
	body[0].push_back(particles[1]->i);

	//head.verts[0] = c2V(particles[1]->x[0], particles[1]->x[1]); //3
	//head.verts[1] = c2V(particles[41]->x[0], particles[41]->x[1]); //8
	//head.verts[2] = c2V(particles[39]->x[0], particles[39]->x[1]); //10
	//head.verts[3] = c2V(particles[12]->x[0], particles[12]->x[1]); //13
	//head.verts[4] = c2V(particles[13]->x[0], particles[13]->x[1]); //12
	//head.verts[5] = c2V(particles[10]->x[0], particles[10]->x[1]); //39
	//head.verts[6] = c2V(particles[8]->x[0], particles[8]->x[1]); //41
	//head.verts[7] = c2V(particles[3]->x[0], particles[3]->x[1]); //1
	//head.count = 8;
	//c2Norms(head.verts, head.norms, head.count);

	//body[0].push_back(particles[1]->i);
	//body[0].push_back(particles[41]->i);
	//body[0].push_back(particles[39]->i);
	//body[0].push_back(particles[12]->i);
	//body[0].push_back(particles[13]->i);
	//body[0].push_back(particles[10]->i);
	//body[0].push_back(particles[8]->i);
	//body[0].push_back(particles[3]->i);

	// torso
	torso.verts[0] = c2V(0.206005, 0.050973); //34
	torso.verts[1] = c2V(0.204401, 0.389945); //163
	torso.verts[2] = c2V(particles[46]->x[0], particles[46]->x[1]); //46
	torso.verts[0] = c2V(particles[52]->x[0], particles[52]->x[1]); //52
	torso.verts[4] = c2V(-0.413897, -0.582149); //186
	torso.verts[5] = c2V(-0.047760, -0.520907); //56
	torso.verts[6] = c2V(0.037287, -0.559413); //187
	torso.verts[7] = c2V(0.366414, -0.575988); //116
	torso.count = 3;
	c2Norms(torso.verts, torso.norms, 3);

	body[1].push_back(34);
	body[1].push_back(163);
	body[1].push_back(46);
	body[1].push_back(52);
	body[1].push_back(186);
	body[1].push_back(56);
	body[1].push_back(187);
	body[1].push_back(116);

	// right arm
	right_arm.verts[0] = c2V(0.206005, 0.050973); //34		torso
	right_arm.verts[1] = c2V(0.204401, 0.389945); //163		torso
	right_arm.verts[2] = c2V(0.526886, 0.445441); //204
	right_arm.verts[3] = c2V(0.720891, 0.367685); //199
	right_arm.verts[4] = c2V(0.717288, 0.203351); //22
	right_arm.verts[5] = c2V(0.506512, 0.110512); //28
	right_arm.count = 6;
	c2Norms(right_arm.verts, right_arm.norms, 6); 

	body[2].push_back(34);
	body[2].push_back(163);
	body[2].push_back(204);
	body[2].push_back(199);
	body[2].push_back(22);
	body[2].push_back(28);

	// left arm

	left_arm.verts[0] = c2V(particles[46]->x[0], particles[46]->x[1]); //46		torso
	left_arm.verts[1] = c2V(particles[52]->x[0], particles[52]->x[1]); //52		torso
	left_arm.verts[2] = c2V(particles[191]->x[0], particles[191]->x[1]); //191
	left_arm.verts[3] = c2V(particles[47]->x[0], particles[47]->x[1]); //47
	left_arm.verts[4] = c2V(particles[207]->x[0], particles[207]->x[1]); //207
	left_arm.verts[5] = c2V(particles[147]->x[0], particles[147]->x[1]); //147
	left_arm.count = 3;
	c2MakePoly(&left_arm);

	body[3].push_back(46);
	body[3].push_back(52);
	body[3].push_back(191);
	body[3].push_back(47);
	body[3].push_back(207);
	body[3].push_back(147);

	// right leg
	right_leg.verts[0] = c2V(particles[210]->x[0], particles[210]->x[1]); //210
	right_leg.verts[1] = c2V(particles[118]->x[0], particles[118]->x[1]); //118
	right_leg.verts[2] = c2V(particles[115]->x[0], particles[115]->x[1]); //115
	right_leg.verts[3] = c2V(particles[116]->x[0], particles[116]->x[1]); //116	torso
	right_leg.verts[4] = c2V(particles[187]->x[0], particles[187]->x[1]); //187	torso
	right_leg.verts[5] = c2V(particles[120]->x[0], particles[120]->x[1]); //120
	right_leg.count = 6;
	c2Norms(right_leg.verts, right_leg.norms, 6);

	body[4].push_back(210);
	body[4].push_back(118);
	body[4].push_back(115);
	body[4].push_back(116);
	body[4].push_back(187);
	body[4].push_back(120);

	// left leg
	left_leg.verts[0] = c2V(particles[67]->x[0], particles[67]->x[1]); //67
	left_leg.verts[1] = c2V(particles[60]->x[0], particles[60]->x[1]); //71
	left_leg.verts[2] = c2V(particles[56]->x[0], particles[56]->x[1]); //104
	left_leg.verts[3] = c2V(particles[186]->x[0], particles[186]->x[1]); //186	torso
	left_leg.verts[4] = c2V(particles[104]->x[0], particles[104]->x[1]); //56		torso
	left_leg.verts[5] = c2V(particles[71]->x[0], particles[71]->x[1]); //60
	left_leg.count = 6;
	c2Norms(left_leg.verts, left_leg.norms, 6);

	body[5].push_back(67);
	body[5].push_back(60);
	body[5].push_back(56);
	body[5].push_back(186);
	body[5].push_back(104);
	body[5].push_back(71);

	//if (c2PolytoPoly(&torso, nullptr, &left_arm, nullptr)) {
	//	cout << "collision" << endl;
	//}

	//c2PolytoPolyManifold(&torso, nullptr, &left_arm, nullptr, &m);
	//cout << "m.count: " << m.count << endl;
	//cout << "collision point: " << m.contact_points[0].x << ", " << m.contact_points[0].y << endl;
	//cout << "collision point: " << m.contact_points[1].x << ", " << m.contact_points[1].y << endl;

	//// debugging
	//cout << body.size() << endl;
	//for (int i = 0; i < body.size(); i++) {
	//	cout << body[i].size() << endl;
	//}


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

	cout << "======= Collision Detection Start =======" << endl;
	//if (c2PolytoPoly(&tri1, nullptr, &tri2, nullptr)) {
	//	cout << "collision" << endl;
	//}
	//if (c2PolytoPoly(&head, nullptr, &right_arm, nullptr)) {
	//	cout << "Head-RightArm Collision" << endl;
	//}
	//if (c2PolytoPoly(&head, nullptr, &left_arm, nullptr)) {
	//	cout << "Head-LeftArm Collision" << endl;
	//	//for (int i = 0; i < body[0].size(); i++) {
	//	//	cout << head.verts[i].x << ", " << head.verts[i].y << endl;
	//	//}
	//}
	//	for (int i = 0; i < body[5].size(); i++) {
	//		cout << left_arm.verts[i].x << ", " << left_arm.verts[i].y << endl;
	//	}
	//}
	//if (c2PolytoPoly(&right_arm, nullptr, &right_leg, nullptr)) {
	//	cout << "RightArm-RightLeg Collision" << endl;
	//}
	//if (c2PolytoPoly(&left_arm, nullptr, &left_leg, nullptr)) {
	//	cout << "LeftArm-LeftLeg Collision" << endl;
	//}
	//if (c2PolytoPoly(&right_leg, nullptr, &left_leg, nullptr)) {
	//	cout << "RightLeg-LeftLeg Collision" << endl;
	//}
	cout << "======= Collision Detection End =======" << endl;
	c2PolytoPolyManifold(&right_leg, nullptr, &left_leg, nullptr, &m);
	cout << "m.count: " << m.count << endl;
	cout << "collision point: " << m.contact_points[0].x << ", " << m.contact_points[0].y << endl;
	cout << "collision point: " << m.contact_points[1].x << ", " << m.contact_points[1].y << endl;
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

	//for (int i = 0; i < debg.size(); i++) {
	//	for (int j = 0; j < debg[i].size(); j++) {
	//		switch (i) {
	//		case 0:
	//			tri1.verts[j] = c2V(particles[debg[i][j]]->x[0], particles[debg[i][j]]->x[1]);
	//			//cout << "case 0" << endl;
	//			break;
	//		case 1:
	//			tri2.verts[j] = c2V(particles[debg[i][j]]->x[0], particles[debg[i][j]]->x[1]);
	//			//cout << "case 1" << endl;
	//			break;
	//		}

	//		//cout << "j = " << j << endl;
	//	}
	//	//cout << "i = " << i << endl;
	//}
	//c2Norms(tri1.verts, tri1.norms, tri1.count);
	//c2Norms(tri2.verts, tri2.norms, tri2.count);

	// cute c2 update
	for (int i = 0; i < body.size(); i++) {
		for (int j = 0; j < body[i].size(); j++) {
			switch(i){
			case 0:
				head.verts[j] = c2V(particles[body[i][j]]->x[0], particles[body[i][j]]->x[1]);
				break;
			case 1:
				torso.verts[j] = c2V(particles[body[i][j]]->x[0], particles[body[i][j]]->x[1]);
				break;
			case 2:
				right_arm.verts[j] = c2V(particles[body[i][j]]->x[0], particles[body[i][j]]->x[1]);
				break;
			case 3:
				left_arm.verts[j] = c2V(particles[body[i][j]]->x[0], particles[body[i][j]]->x[1]);
				break;
			case 4:
				right_leg.verts[j] = c2V(particles[body[i][j]]->x[0], particles[body[i][j]]->x[1]);
				break;
			case 5:		
				left_leg.verts[j] = c2V(particles[body[i][j]]->x[0], particles[body[i][j]]->x[1]);
				break;
			}
		}
	}
	c2Norms(head.verts, head.norms, head.count);
	c2Norms(torso.verts, torso.norms, 8);
	c2Norms(right_arm.verts, right_arm.norms, 6);
	c2MakePoly(&left_arm);
	c2Norms(right_leg.verts, right_leg.norms, right_leg.count);
	c2Norms(left_leg.verts, left_leg.norms, left_leg.count);

	// debugging
	//for (int i = 0; i < body.size(); i++) {
	//	for (int j = 0; j < body[i].size(); j++) {
	//		cout << "Particle Position: \n" << particles[body[i][j]]->x << endl;
	//	}

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

	//glLineWidth(2.0f);
	//glColor3d(1.0, 0.0, 0.0);
	//glBegin(GL_LINES);
	//for (int i = 0; i < tri1.count; i++) {
	//	if (i == 2) {
	//		glVertex2f(tri1.verts[i].x, tri1.verts[i].y);
	//		glVertex2f(tri1.verts[0].x, tri1.verts[0].y);
	//	}
	//	else {
	//		glVertex2f(tri1.verts[i].x, tri1.verts[i].y);
	//		glVertex2f(tri1.verts[i + 1].x, tri1.verts[i + 1].y);
	//	}
	//}
	//glEnd();

	//glLineWidth(2.0f);
	//glColor3d(1.0, 0.0, 0.0);
	//glBegin(GL_LINES);
	//for (int i = 0; i < tri2.count; i++) {
	//	if (i == 2) {
	//		glVertex2f(tri2.verts[i].x, tri2.verts[i].y);
	//		glVertex2f(tri2.verts[0].x, tri2.verts[0].y);
	//	}
	//	else {
	//		glVertex2f(tri2.verts[i].x, tri2.verts[i].y);
	//		glVertex2f(tri2.verts[i + 1].x, tri2.verts[i + 1].y);
	//	}
	//}
	//glEnd();

	//glLineWidth(2.0f);
	//glColor3d(1.0, 0.0, 0.0);
	//glBegin(GL_LINES);
	//for (int i = 0; i < head.count; i++) {
	//	if (i == (head.count - 1)) {
	//		glVertex2f(head.verts[i].x, head.verts[i].y);
	//		glVertex2f(head.verts[0].x, head.verts[0].y);
	//	}
	//	else {
	//		glVertex2f(head.verts[i].x, head.verts[i].y);
	//		glVertex2f(head.verts[i + 1].x, head.verts[i + 1].y);
	//	}
	//}
	//glEnd();

	//glLineWidth(2.0f);
	//glColor3d(1.0, 0.0, 0.0);
	//glBegin(GL_LINES);
	//for (int i = 0; i < torso.count; i++) {
	//	if (i == 7) {
	//		glVertex2f(torso.verts[i].x, torso.verts[i].y);
	//		glVertex2f(torso.verts[0].x, torso.verts[0].y);
	//	}
	//	else {
	//		glVertex2f(torso.verts[i].x, torso.verts[i].y);
	//		glVertex2f(torso.verts[i + 1].x, torso.verts[i + 1].y);
	//	}
	//}
	//glEnd();

	//glLineWidth(2.0f);
	//glColor3d(1.0, 0.0, 0.0);
	//glBegin(GL_LINES);
	//for (int i = 0; i < right_arm.count; i++) {
	//	if (i == 5) {
	//		glVertex2f(right_arm.verts[i].x, right_arm.verts[i].y);
	//		glVertex2f(right_arm.verts[0].x, right_arm.verts[0].y);
	//	}
	//	else {
	//		glVertex2f(right_arm.verts[i].x, right_arm.verts[i].y);
	//		glVertex2f(right_arm.verts[i + 1].x, right_arm.verts[i + 1].y);
	//	}
	//}
	//glEnd();

	//glLineWidth(2.0f);
	//glColor3d(1.0, 0.0, 0.0);
	//glBegin(GL_LINES);
	//for (int i = 0; i < left_arm.count; i++) {
	//	if (i == 5) {
	//		glVertex2f(left_arm.verts[i].x, left_arm.verts[i].y);
	//		glVertex2f(left_arm.verts[0].x, left_arm.verts[0].y);
	//	}
	//	else {
	//		glVertex2f(left_arm.verts[i].x, left_arm.verts[i].y);
	//		glVertex2f(left_arm.verts[i + 1].x, left_arm.verts[i + 1].y);
	//	}
	//}
	//glEnd();

	glLineWidth(2.0f);
	glColor3d(1.0, 0.0, 0.0);
	glBegin(GL_LINES);
	for (int i = 0; i < right_leg.count; i++) {
		if (i == 5) {
			glVertex2f(right_leg.verts[i].x, right_leg.verts[i].y);
			glVertex2f(right_leg.verts[0].x, right_leg.verts[0].y);
		}
		else {
			glVertex2f(right_leg.verts[i].x, right_leg.verts[i].y);
			glVertex2f(right_leg.verts[i + 1].x, right_leg.verts[i + 1].y);
		}
	}
	glEnd();

	glLineWidth(2.0f);
	glColor3d(1.0, 0.0, 0.0);
	glBegin(GL_LINES);
	for (int i = 0; i < left_leg.count; i++) {
		if (i == 5) {
			glVertex2f(left_leg.verts[i].x, left_leg.verts[i].y);
			glVertex2f(left_leg.verts[0].x, left_leg.verts[0].y);
		}
		else {
			glVertex2f(left_leg.verts[i].x, left_leg.verts[i].y);
			glVertex2f(left_leg.verts[i + 1].x, left_leg.verts[i + 1].y);
		}
	}
	glEnd();

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