#pragma once
#ifndef TRIANGLE_H
#define TRIANGLE_H

#include <memory>

class Particle;

class Triangle {
public:
	Triangle(const std::shared_ptr<Particle> p0, const std::shared_ptr<Particle> p1, const std::shared_ptr<Particle> p2);
	virtual ~Triangle();

	std::shared_ptr<Particle> p0;
	std::shared_ptr<Particle> p1;
	std::shared_ptr<Particle> p2;

	double E;
};

#endif