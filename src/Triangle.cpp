#include "Triangle.h"
#include "Particle.h"

#include <memory>

using namespace std;
using namespace Eigen;

Triangle::Triangle(shared_ptr<Particle> p0, shared_ptr<Particle> p1, shared_ptr<Particle> p2) :
	E(1.0)
{
	assert(p0);
	assert(p1);
	assert(p2);
	assert(p0 != p1);
	assert(p0 != p2);
	assert(p1 != p2);

	this->p0 = p0;
	this->p1 = p1;
	this->p2 = p2;
	Vector2d x0 = p0->x;
	Vector2d x1 = p1->x;
	Vector2d x2 = p2->x;


}

Triangle::~Triangle() {

}