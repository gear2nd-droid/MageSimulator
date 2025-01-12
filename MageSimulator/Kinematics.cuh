#pragma once
#ifndef KINEMATICS
#define KINEMATICS

#include "Structs.cuh"
#include "Utility.cuh"

#define _USE_MATH_DEFINES
#include <math.h>

#define DISTANCE_FIRST_DIV 10
#define DISTANCE_TARGET 0.05

class Kinematics
{
public:
	Kinematics();
	~Kinematics();
	virtual GcodeItem InverseKinematics4Item(PathItem path, GcodeItem prev);
	virtual float* ForwardKinematics(float x, float y, float z, float a, float b, float c);
	virtual PathItem ForwardKinematics4Item(GcodeItem gcode);
	virtual float CalcDistance(GcodeItem prev, GcodeItem now);
	virtual void GetRotationMatrix(float a, float b, float c, float* mat);
	virtual void DisplayStart(CameraParam camPrm, PrintItem item);
};

class CoreXYBC : public Kinematics
{
public:
	CoreXYBC();
	~CoreXYBC();
	GcodeItem InverseKinematics4Item(PathItem path, GcodeItem prev) override;
	float* ForwardKinematics(float x, float y, float z, float a, float b, float c) override;
	PathItem ForwardKinematics4Item(GcodeItem gcode) override;
	float CalcDistance(GcodeItem prev, GcodeItem now) override;
	void GetRotationMatrix(float a, float b, float c, float* mat) override;
	void DisplayStart(CameraParam camPrm, PrintItem item) override;
	float minB;
	float maxB;
	float minC;
	float maxC;
};

class CoreXY : public Kinematics
{
public:
	CoreXY();
	~CoreXY();
	GcodeItem InverseKinematics4Item(PathItem path, GcodeItem prev) override;
	float* ForwardKinematics(float x, float y, float z, float a, float b, float c) override;
	PathItem ForwardKinematics4Item(GcodeItem gcode) override;
	float CalcDistance(GcodeItem prev, GcodeItem now) override;
	void GetRotationMatrix(float a, float b, float c, float* mat) override;
	void DisplayStart(CameraParam camPrm, PrintItem item) override;
};

class BedSlingerY : public Kinematics
{
public:
	BedSlingerY();
	~BedSlingerY();
	GcodeItem InverseKinematics4Item(PathItem path, GcodeItem prev) override;
	float* ForwardKinematics(float x, float y, float z, float a, float b, float c) override;
	PathItem ForwardKinematics4Item(GcodeItem gcode) override;
	float CalcDistance(GcodeItem prev, GcodeItem now) override;
	void GetRotationMatrix(float a, float b, float c, float* mat) override;
	void DisplayStart(CameraParam camPrm, PrintItem item) override;
};

class Delta : public Kinematics
{
public:
	Delta();
	~Delta();
	GcodeItem InverseKinematics4Item(PathItem path, GcodeItem prev) override;
	float* ForwardKinematics(float x, float y, float z, float a, float b, float c) override;
	PathItem ForwardKinematics4Item(GcodeItem gcode) override;
	float CalcDistance(GcodeItem prev, GcodeItem now) override;
	void GetRotationMatrix(float a, float b, float c, float* mat) override;
	void DisplayStart(CameraParam camPrm, PrintItem item) override;
};





#endif