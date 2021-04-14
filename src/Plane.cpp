#include <iostream>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>


#include "Plane.h"
#include "MatrixStack.h"
#include "Program.h"
#include "GLSL.h"
#include "Shape.h"

using namespace std;
using namespace Eigen;

Plane::Plane() : 
	xmax(0.0),
	xmin(0.0),
	ymax(0.0),
	ymin(0.0),
	flag(0)
{

}

Plane::Plane(float x1, float x2, float y1, float y2) :
	xmax(x1),
	xmin(x2),
	ymax(y1), 
	ymin(y2),
	flag(0)
{

}

Plane::~Plane() {

}

void Plane::init() {
	if (ymin == 0) {
		flag = 1;
	}
	else if (ymax == 0) {
		flag = 2;
	}
	else if (xmin == 0) {
		flag = 3;
	}
	else if (xmax == 0) {
		flag = 4;
	}
}

void Plane::draw(shared_ptr<MatrixStack> MV, const shared_ptr<Program> p) const {
	MV->pushMatrix();
	glUniformMatrix4fv(p->getUniform("MV"), 1, GL_FALSE, glm::value_ptr(MV->topMatrix()));
	glLineWidth(2.0f);
	glColor3d(0.2, 0.2, 0.2);
	glBegin(GL_LINES);
	switch (flag) {
	case 1: glVertex2d(xmin, ymax);
		glVertex2d(xmax, ymax);
		
		break;
	case 2: glVertex2d(xmin, ymin);
		glVertex2d(xmax, ymin);
		break;
	case 3: glVertex2d(xmax, ymin);
		glVertex2d(xmax, ymax);
		break;
	case 4: glVertex2d(xmin, ymin);
		glVertex2d(xmin, ymax);
		break;
	}

	glEnd();
	MV->popMatrix();
}
