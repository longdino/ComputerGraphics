#pragma once
#ifndef Particle_H
#define Particle_H

#include <vector>
#include <memory>

#define EIGEN_DONT_ALIGN_STATICALLY
#include <Eigen/Dense>

class Shape;
class Program;
class MatrixStack;

class Particle
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	Particle();
	Particle(const std::shared_ptr<Shape> shape, double r, Eigen::Vector2d x);
	virtual ~Particle();
	void tare();
	void reset();
	void init();
	void draw(std::shared_ptr<MatrixStack> MV, const std::shared_ptr<Program> p) const;
	
	double r; // radius
	double m; // mass
	int i;  // starting index
	Eigen::Vector2d X; // initial position
	Eigen::Vector2d x;  // position
	Eigen::Vector2d v;  // velocity
	Eigen::Vector2d v0;
	bool fixed;
	
private:
	const std::shared_ptr<Shape> sphere;
};

#endif
