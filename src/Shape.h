#pragma once
#ifndef SHAPE_H
#define SHAPE_H

#include <string>
#include <vector>
#include <memory>

class Program;

/**
 * A shape defined by a list of triangles
 * - posBuf should be of length 3*ntris
 * - norBuf should be of length 3*ntris (if normals are available)
 * - texBuf should be of length 2*ntris (if texture coords are available)
 * posBufID, norBufID, and texBufID are OpenGL buffer identifiers.
 */
class Shape
{
public:
	Shape();
	virtual ~Shape();
	void loadMesh(const std::string &meshName);
	void load(const std::string& meshName);
	void toLocal();
	void toWorld();
	void init();
	void draw(const std::shared_ptr<Program> prog) const;
	void draw(int h_pos, int h_tex) const;
	std::vector<float> getPos();
	std::vector<float> getTex();
	std::vector<float> getTri();
	std::vector<float> getElem();
	
private:
	std::vector<float> posBuf;
	std::vector<float> norBuf;
	std::vector<float> texBuf;
	std::vector<float> elemBuf;
	std::vector<float> triBuf;
	std::vector<float> posLocalBuf;
	std::vector<float> tileIndexBuf;
	unsigned posBufID;
	unsigned norBufID;
	unsigned texBufID;
	unsigned elemBufID;
};

#endif
