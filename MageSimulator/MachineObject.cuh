#pragma once
#ifndef MACHINE_OBJECT
#define MACHINE_OBJECT


#include "Structs.cuh"
#include "PrintObject.cuh"

#define _USE_MATH_DEFINES
#include <math.h>
#include <tinyxml2.h>
#include <string>
using namespace std;

#define HEAD_FACE_COLOR_R 0.5
#define HEAD_FACE_COLOR_G 0.5
#define HEAD_FACE_COLOR_B 1.0
#define HEAD_FACE_TRANS 0.5
#define GANTRY_FACE_COLOR_R 1.0
#define GANTRY_FACE_COLOR_G 0.5
#define GANTRY_FACE_COLOR_B 0.5
#define GANTRY_FACE_TRANS 0.5
#define BED_FACE_COLOR_R 0.5
#define BED_FACE_COLOR_G 1.0
#define BED_FACE_COLOR_B 0.5
#define BED_FACE_TRANS 0.5

#define HEAD_EDGE_COLOR_R 0.2
#define HEAD_EDGE_COLOR_G 0.2
#define HEAD_EDGE_COLOR_B 1.0
#define HEAD_EDGE_TRANS 0.5
#define GANTRY_EDGE_COLOR_R 1.0
#define GANTRY_EDGE_COLOR_G 0.2
#define GANTRY_EDGE_COLOR_B 0.2
#define GANTRY_EDGE_TRANS 0.5
#define BED_EDGE_COLOR_R 0.2
#define BED_EDGE_COLOR_G 1.0
#define BED_EDGE_COLOR_B 0.2
#define BED_EDGE_TRANS 0.5

class CollisionObject
{
public:
	CollisionObject();
	CollisionObject(MachineType type);
	~CollisionObject();
	virtual std::vector<Mesh> move(PrintItem item, Kinematics* kin, bool isdraw);
protected:
	MachineType type;
};

class CollisionCylinder : public CollisionObject
{
public:
	CollisionCylinder();
	CollisionCylinder(MachineType type, tinyxml2::XMLElement* xml);
	~CollisionCylinder();
	std::vector<Mesh> move(PrintItem item, Kinematics* kin, bool isdraw) override;
protected:
	float lower_radius;
	float upper_radius;
	float height;
	int div;
	float lowerX;
	float lowerY;
	float lowerZ;
public:
	float** vertices;
	int** indices;
	int triNum;
};

class CollisionBox : public CollisionObject
{
public:
	CollisionBox();
	CollisionBox(MachineType type, tinyxml2::XMLElement* xml);
	~CollisionBox();
	std::vector<Mesh> move(PrintItem item, Kinematics* kin, bool isdraw) override;
protected:
	float centerX;
	float centerY;
	float centerZ;
	float sizeX;
	float sizeY;
	float sizeZ;
public:
	float** vertices;
	int** indices;
};



#endif