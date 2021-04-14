#pragma once
#ifndef PLANE_H
#define PLANE_H

#include <vector>
#include <memory>

#define EIGEN_DONT_ALIGN_STATICALLY
#include <Eigen/Dense>
#include <Eigen/Sparse>

class Shape;
class Program;
class MatrixStack;
class Texture;

class Plane {
public:
	Plane();
	Plane(float x1, float x2, float y1, float y2);
	virtual ~Plane();

	void init();
	void draw(std::shared_ptr<MatrixStack> MV, const std::shared_ptr<Program> p) const;

	float xmax;
	float xmin;
	float ymax;
	float ymin;
	int flag;
private:
};

#endif