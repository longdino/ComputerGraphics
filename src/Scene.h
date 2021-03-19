#pragma once
#ifndef Scene_H
#define Scene_H

#include <vector>
#include <memory>
#include <string>

#define EIGEN_DONT_ALIGN_STATICALLY
#include <Eigen/Dense>

class Cloth;
class Particle;
class MatrixStack;
class Program;
class Shape;
class Texture;
class Ginger;
class Plane;

class Scene
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	Scene();
	virtual ~Scene();
	
	void load(const std::string &RESOURCE_DIR);
	void init();
	void tare();
	void reset();
	void step();
	
	void draw(std::shared_ptr<MatrixStack> MV, const std::shared_ptr<Program> prog) const;
	
	double getTime() const { return t; }
	
private:
	double t;
	double h;
	Eigen::Vector2d grav;
	
	double nu;
	double young;
	double mu;
	double lambda;
	Eigen::MatrixXd dX;
	double a;
	Eigen::MatrixXd dXinv;
	
	// load obj files
	std::shared_ptr<Shape> sphereShape;
	std::shared_ptr<Shape> gingerman;
	std::shared_ptr<Shape> square;
	std::shared_ptr<Shape> floorMesh;

	// load texture
	std::shared_ptr<Texture> texture;

	// particle
	std::vector< std::shared_ptr<Particle> > spheres;

	// FEM applied
	std::shared_ptr<Cloth> cloth;
	std::shared_ptr<Ginger> ginger;
	std::shared_ptr<Ginger> squareMesh;
	std::shared_ptr<Ginger> floor;

	// walls
	std::shared_ptr<Plane> wall1;
	std::shared_ptr<Plane> wall2;
	std::shared_ptr<Plane> ceiling;
	std::shared_ptr<Plane> flr;
	std::vector< std::shared_ptr<Plane> > planes;

	// test
	std::shared_ptr<Ginger> ginger2;

};

#endif
