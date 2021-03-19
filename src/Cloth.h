#pragma once
#ifndef Cloth_H
#define Cloth_H

#include <vector>
#include <memory>

#define EIGEN_DONT_ALIGN_STATICALLY
#include <Eigen/Dense>
#include <Eigen/Sparse>

class Particle;
class Spring;
class MatrixStack;
class Program;
class Triangle;

class Cloth
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	Cloth(int rows, int cols,
		  const Eigen::Vector2d &x00,
		  const Eigen::Vector2d &x01,
		  const Eigen::Vector2d &x10,
		  const Eigen::Vector2d &x11,
		  double mass,
		  double stiffness);
	//Cloth(const std::shared_ptr<Shape> c, const std::vector<float> position, const std::vector<float> texture, const std::vector<float> triangle, double mass, double stiffness);
	virtual ~Cloth();
	
	void tare();
	void reset();
	void updatePosNor();
	void step(double h, double mu, double lambda, const Eigen::Vector2d &grav, const std::vector< std::shared_ptr<Particle> > spheres);
	
	void init();
	void draw(std::shared_ptr<MatrixStack> MV, const std::shared_ptr<Program> p) const;
	
private:
	int rows;
	int cols;
	int n;
	std::vector< std::shared_ptr<Particle> > particles;
	std::vector< std::shared_ptr<Spring> > springs;
	std::vector< std::shared_ptr<Triangle> > triangles;
	
	Eigen::VectorXd v;
	Eigen::VectorXd f;
	Eigen::MatrixXd M;
	Eigen::MatrixXd K;

	Eigen::VectorXd fs;
	Eigen::MatrixXd Ks;
	
	std::vector<unsigned int> eleBuf;
	std::vector<float> posBuf;
	std::vector<float> norBuf;
	std::vector<float> texBuf;
	unsigned eleBufID;
	unsigned posBufID;
	unsigned norBufID;
	unsigned texBufID;
};

#endif
