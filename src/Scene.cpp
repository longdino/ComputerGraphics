#include <iostream>

#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "Scene.h"
#include "Particle.h"
#include "Cloth.h"
#include "Shape.h"
#include "Program.h"
#include "MatrixStack.h"
#include "Texture.h"
#include "Ginger.h"
#include "Plane.h"

using namespace std;
using namespace Eigen;

Scene::Scene() :
	t(0.0),
	h(1e-2),
	grav(0.0, 0.0)
{
}

Scene::~Scene()
{
}

void Scene::load(const string &RESOURCE_DIR)
{
	// Units: meters, kilograms, seconds
	h = 5e-3;

	grav << 0.0, -9.8;

	double mass = 0.1;
	double stiffness = 1e1;
	double radius = 0.02;
	//int rows = 3;
	//int cols = 3;
	//Vector2d x00(-0.5, 0.5);
	//Vector2d x01(0.5, 0.5);
	//Vector2d x10(-0.5, -0.5);
	//Vector2d x11(0.5, -0.5);
	//cloth = make_shared<Cloth>(rows, cols, x00, x01, x10, x11, mass, stiffness);

	// snh variables
	nu = 0.45;
	young = 1e1;
	mu = young / (2 * (1 + nu));
	lambda = young * nu / ((1 + nu) * (1 - 2 * nu));

	// load texture
	texture = make_shared<Texture>();
	texture->setFilename(RESOURCE_DIR + "wood_tex.jpg");

	// wall lines
	ceiling = make_shared<Plane>(1.5, -1.5, 1.5, 0);
	planes.push_back(ceiling);
	wall1 = make_shared<Plane>(1.5, 0, 1.5, -1.5);
	planes.push_back(wall1);
	wall2 = make_shared<Plane>(0, -1.5, 1.5, -1.5);
	planes.push_back(wall2);
	flr = make_shared<Plane>(1.5, -1.5, 0, -1.5);
	planes.push_back(flr);

	// floor mesh
	floorMesh = make_shared<Shape>();
	floorMesh->loadMesh(RESOURCE_DIR + "floor.obj");

	vector<float> flrPos = floorMesh->getPos();
	vector<float> flrTex = floorMesh->getTex();
	vector<float> flrElem = floorMesh->getElem();

	// translate position
	for (int i = 0; i < flrPos.size(); i+=3) {
		flrPos[i] -= 2.5;
		flrPos[i + 1] -= 2.5;
	}

	floor = make_shared<Ginger>(floorMesh, flrPos, flrTex, flrElem, mass, stiffness, radius);

	// Gingerman
	gingerman = make_shared<Shape>();
	gingerman->loadMesh(RESOURCE_DIR + "man.obj");
	
	vector<float> tex = gingerman->getTex();
	vector<float> position = gingerman->getPos();
	vector<float> elem = gingerman->getElem();

	radius = 0.02;

	ginger = make_shared<Ginger>(gingerman, position, tex, elem, mass, stiffness, radius);

	// Square mesh
	//square = make_shared<Shape>();
	//square->loadMesh(RESOURCE_DIR + "square.obj");

	//vector<float> sqTex = square->getTex();
	//vector<float> sqPos = square->getPos();
	//vector<float> sqEle = square->getElem();

	//squareMesh = make_shared<Ginger>(square, sqPos, sqTex, sqEle, mass, stiffness, radius);
}

void Scene::init()
{
	texture->init();
	texture->setUnit(0);
	ginger->init();

	ceiling->init();
	wall1->init();
	wall2->init();
	flr->init();
	//floor->init();
	//cloth->init();
}

void Scene::tare()
{
	ginger->tare();
	//cloth->tare();

	//floor->tare();
}

void Scene::reset()
{
	t = 0.0;
	ginger->reset();
	//cloth->reset();

	//floor->reset();
}

void Scene::step()
{
	t += h;
	ginger->step(h, mu, lambda, grav, flr);
	//cloth->step(h, mu, lambda, grav, spheres);
}

void Scene::draw(shared_ptr<MatrixStack> MV, const shared_ptr<Program> prog) const
{
	//cloth->draw(MV, prog);
	texture->bind(prog->getUniform("colorTexture"));
	ginger->draw(MV, prog);
	texture->unbind();

	ceiling->draw(MV, prog);
	wall1->draw(MV, prog);
	wall2->draw(MV, prog);
	flr->draw(MV, prog);
	//floor->draw(MV, prog);
}
