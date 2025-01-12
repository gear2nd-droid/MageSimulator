#pragma once
#ifndef STRUCTS
#define STRUCTS

#include <string>
#define _USE_MATH_DEFINES
#include <math.h>

typedef struct caemraparam
{
	float cameraDistance = 150.0f;
	float cameraAngleX = M_PI * 1.0 / 4.0;
	float cameraAngleZ = 0.0;
	float targetPosX = 0.0f;
	float targetPosY = 0.0f;
	float targetPosZ = 0.0f;
}CameraParam;

typedef struct mesh
{
	float pnt0[3];
	float pnt1[3];
	float pnt2[3];
	int id;
}Mesh;

typedef struct ray
{
	float org[3];
	float tar[3];
	int id;
}Ray;

typedef struct collisionset
{
	float pnt[3];
	int meshId;
	int rayId;
	bool colFlag;
}CollisionSet;

typedef struct gcode
{
	bool gORm; // T:Gcode,F:Mcode
	int codeVal;
	float x;
	float y;
	float z;
	float a;
	float b;
	float c;
	float distance;
	float e;
	float f;
	float s;
	std::string comment;
	float thick;
	float width;
	int blockIdx;
	int layerIdx;
	int loopIdx;
	int curveIdx;
	int lineIdx;
}GcodeItem;

typedef enum printtype
{
	Move = 0,
	OuterWall = 1,
	InnerWall = 2,
	Infill = 3,
	Support = 4,
	Print = 5
}PrintType;

typedef struct printitem
{
	PrintType type;
	int time;
	float stPnt[3];
	float edPnt[3];
	float size[3];
	float center[3];
	float tip[3];
	float dir[3];
	float vertex[8][3];
	float faceNorm[6][3];
	float quatNorm[3];
	float quatAngle;
	float headNorm[3];
	float rot[3];
	float rotMat[3 * 3];
	float invRotMat[3 * 3];
	float bbox[2][3];
	Ray ray[12];
	Mesh mesh[12];
	float cmdVal[8]; // x,y,z,a,b,c,e,f
	float stMoveVal[6];
	float edMoveVal[6];
	int blockIdx;
	int layerIdx;
	int lineIdx;
}PrintItem;

const int BoxVertex[6][4] = {
	{0, 1, 2, 3},
	{4, 5, 6, 7},
	{0, 3, 5, 4},
	{3, 2, 6, 5},
	{2, 1, 7, 6},
	{1, 7, 4, 0}
};

typedef enum pathtype
{
	PrintMiddle = 1,
	PrintStart = 2,
	PrintEnd = 3,
	OuterWallMiddle = 11,
	OuterWallStart = 12,
	OuterWallEnd = 13,
	InnerWallMiddle = 21,
	InnerWallStart = 22,
	InnerWallEnd = 23,
	InfillMiddle = 31,
	InfillStart = 32,
	InfillEnd = 33,
	SupportMiddle = 41,
	SupportStart = 42,
	SupportEnd = 43,
	Saving = 51,
	None = 0
}PathType;

typedef struct pathitem
{
	PathType type;
	float pnt[3];
	float norm[3];
	float thick;
	float width;
	int blockIdx;
	int layerIdx;
	int loopIdx;
	int curveIdx;
	float cmdVal[8]; // x,y,z,a,b,c,e,f
	float moveVal[6]; // x,y,z,a,b,c
	int lineIdx;
}PathItem;

typedef enum machinetype
{
	Head = 0,
	XGantry = 1,
	YGantry = 2,
	Bed = 3
}MachineType;

#endif