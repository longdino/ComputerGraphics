#include <iostream>

#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "Particle.h"
#include "Shape.h"
#include "Program.h"
#include "MatrixStack.h"

using namespace std;

Particle::Particle() :
	r(1.0),
	m(1.0),
	i(-1),
	x(0.0, 0.0),
	v(0.0, 0.0),
	fixed(true)
{
	
}

Particle::Particle(const shared_ptr<Shape> s, double r, Eigen::Vector2d pos) :
	r(r),
	m(1.0),
	i(-1),
	x(pos(0), pos(1)),
	v(0.0, 0.0),
	fixed(true),
	sphere(s)
{
	
}

Particle::~Particle()
{
}

void Particle::tare()
{
	X = x;
	v0 = v;
}

void Particle::reset()
{
	x = X;
	v = v0;
}

void Particle::init() {
	if (sphere) {
		sphere->init();
	}
}

void Particle::draw(shared_ptr<MatrixStack> MV, const shared_ptr<Program> prog) const
{
	if(sphere) {
		MV->pushMatrix();
		MV->translate(x(0), x(1), 0.0);
		MV->scale(r);
		glUniformMatrix4fv(prog->getUniform("MV"), 1, GL_FALSE, glm::value_ptr(MV->topMatrix()));
		sphere->draw(prog);
		MV->popMatrix();
	}
}
