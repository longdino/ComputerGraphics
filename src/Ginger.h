#pragma once
#ifndef GINGER_H
#define GINGER_H

#include <vector>
#include <memory>

#define EIGEN_DONT_ALIGN_STATICALLY
#include <Eigen/Dense>
#include <Eigen/Sparse>

class Particle;
class Triangle;
class MatrixStack;
class Program;
class Texture;
class Shape;
class Snh;
class Plane;

class Ginger {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	Ginger(const std::shared_ptr<Shape> g, const std::vector<float> position, const std::vector<float> texture, const std::vector<float> triangle, double mass, double stiffness, double radius);
	virtual ~Ginger();

	void tare();
	void reset();
	void updatePos();
	//void step(double h, double mu, double lambda, const Eigen::Vector2d& grav, const std::shared_ptr<Ginger> floor);
	void step(double h, double mu, double lambda, const Eigen::Vector2d& grav, const std::shared_ptr<Plane> floor);

	void init();
	void draw(std::shared_ptr<MatrixStack> MV, const std::shared_ptr<Program> p) const;

private:
	int size;
	int n;
	std::vector<std::shared_ptr<Particle>>particles;
	std::vector<std::shared_ptr<Triangle>>triangles;

	Eigen::VectorXd v;
	Eigen::VectorXd f;
	Eigen::MatrixXd M;
	Eigen::MatrixXd K;

	Eigen::VectorXd fs;
	Eigen::MatrixXd Ks;

	std::vector<unsigned int> eleBuf;
	std::vector<float> posBuf;
	std::vector<float> texBuf;
	unsigned eleBufID;
	unsigned posBufID;
	unsigned texBufID;

};


#endif