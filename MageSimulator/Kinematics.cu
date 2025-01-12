#include "Kinematics.cuh"

Kinematics::Kinematics()
{
}

Kinematics::~Kinematics()
{
}

GcodeItem Kinematics::InverseKinematics4Item(PathItem path, GcodeItem prev)
{
	GcodeItem ret;
	return ret;
}

float* Kinematics::ForwardKinematics(float x, float y, float z, float a, float b, float c)
{
	float ret[3];
	return ret;
}

PathItem Kinematics::ForwardKinematics4Item(GcodeItem gcode)
{
	PathItem ret;
	return ret;
}

float Kinematics::CalcDistance(GcodeItem prev, GcodeItem now)
{
	return 0.0;
}

void Kinematics::GetRotationMatrix(float a, float b, float c, float* mat)
{
	mat[0] = 1.0;
	mat[1] = 0.0;
	mat[2] = 0.0;
	mat[3] = 0.0;
	mat[4] = 1.0;
	mat[5] = 0.0;
	mat[6] = 0.0;
	mat[7] = 0.0;
	mat[8] = 1.0;
}

void Kinematics::DisplayStart(CameraParam camPrm, PrintItem item)
{

}

CoreXYBC::CoreXYBC()
{
}

CoreXYBC::~CoreXYBC()
{
}

GcodeItem CoreXYBC::InverseKinematics4Item(PathItem path, GcodeItem prev)
{
	GcodeItem ret;
	float tilt = -acos(path.norm[2]);
	float rot;
	if (path.norm[0] == 0.0)
	{
		if (sin(tilt) == 0.0)
		{
			rot = prev.c;
		}
		else
		{
			float buf1 = abs(prev.c - M_PI / 2.0);
			float buf2 = abs(prev.c + M_PI / 2.0);
			if (buf1 < buf2)
			{
				rot = M_PI / 2.0;
			}
			else
			{
				rot = -M_PI / 2.0;
			}
		}
	}
	else
	{
		rot = -atan2(path.norm[1], path.norm[0]);
	}
	// pos
	float bx1 = cos(rot) * path.pnt[0] - sin(rot) * path.pnt[1];
	float by1 = sin(rot) * path.pnt[0] + cos(rot) * path.pnt[1];
	float bz1 = path.pnt[2];
	float bx2 = cos(tilt) * bx1 + sin(tilt) * bz1;
	float by2 = by1;
	float bz2 = -sin(tilt) * bx1 + cos(tilt) * bz1;
	// rot_sel
	float buf_base = ceil(prev.c / M_PI / 2.0);
	float buf_11 = (buf_base - 1) * M_PI * 2.0 + rot;
	float buf_12 = buf_base * M_PI * 2.0 + rot;
	float buf_13 = (buf_base + 1) * M_PI * 2.0 + rot;
	float dist_1 = sqrt((buf_11 - prev.c) * (buf_11 - prev.c));
	float dist_2 = sqrt((buf_12 - prev.c) * (buf_12 - prev.c));
	float dist_3 = sqrt((buf_13 - prev.c) * (buf_13 - prev.c));
	if ((dist_1 <= dist_2) && (dist_1 <= dist_3))
	{
		rot = buf_11;
	}
	else if ((dist_2 <= dist_1) && (dist_2 <= dist_3))
	{
		rot = buf_12;
	}
	else
	{
		rot = buf_13;
	}
	// value set
	ret.x = bx2;
	ret.y = by2;
	ret.z = bz2;
	ret.a = 0.0;
	ret.b = tilt;
	ret.c = rot;
	if (path.type == PathType::OuterWallStart || PathType::InnerWallStart || 
		PathType::InnerWallStart || PathType::SupportStart || PathType::Saving)
	{
		ret.gORm = true;
		ret.codeVal = 0;
	}
	else
	{
		ret.gORm = true;
		ret.codeVal = 1;
	}
	// distance
	ret.distance = CalcDistance(prev, ret);

	return ret;
}

float* CoreXYBC::ForwardKinematics(float x, float y, float z, float a, float b, float c)
{
	float ret[3];
	float bx = x * cos(-b) + z * sin(-b);
	float by = y;
	float bz = -x * sin(-b) + z * cos(-b);
	ret[0] = bx * cos(-c) - by * sin(-c);
	ret[1] = bx * sin(-c) + by * cos(-c);
	ret[2] = bz;
	return ret;
}

PathItem CoreXYBC::ForwardKinematics4Item(GcodeItem gcode)
{
	PathItem ret;
	// pos
	float bx = gcode.x * cos(-gcode.b) + gcode.z * sin(-gcode.b);
	float by = gcode.y;
	float bz = -gcode.x * sin(-gcode.b) + gcode.z * cos(-gcode.b);
	ret.pnt[0] = bx * cos(-gcode.c) - by * sin(-gcode.c);
	ret.pnt[1] = bx * sin(-gcode.c) + by * cos(-gcode.c);
	ret.pnt[2] = bz;
	// norm
	ret.norm[0] = cos(-gcode.c) * sin(-gcode.b);
	ret.norm[1] = sin(-gcode.c) * sin(-gcode.b);
	ret.norm[2] = cos(-gcode.b);
	// other
	ret.width = gcode.width;
	ret.thick = gcode.thick;
	return ret;
}

float CoreXYBC::CalcDistance(GcodeItem prev, GcodeItem now)
{
	int div = 1;
	// calc div
	float dist = 0.0;
	float stx1 = prev.x * cos(-prev.b) + prev.z * sin(-prev.b);
	float sty1 = prev.y;
	float stz1 = -prev.x * sin(-prev.b) + prev.z * cos(-prev.b);
	float stx2 = stx1 * cos(-prev.c) - sty1 * sin(-prev.c);
	float sty2 = stx1 * sin(-prev.c) + sty1 * cos(-prev.c);
	float stz2 = stz1;
	float bx4 = stx2;
	float by4 = sty2;
	float bz4 = sty2;
	for (int i = 0; i < div; i++)
	{
		float bx1 = (now.x - prev.x) * (i + 1) / div + prev.x;
		float by1 = (now.y - prev.y) * (i + 1) / div + prev.y;
		float bz1 = (now.z - prev.z) * (i + 1) / div + prev.z;
		float brot = (now.c - prev.c) * (i + 1) / div + prev.c;
		float btilt = (now.b - prev.b) * (i + 1) / div + prev.b;
		float bx2 = bx1 * cos(-btilt) + bz1 * sin(-btilt);
		float by2 = by1;
		float bz2 = -bx1 * sin(-btilt) + bz1 * cos(-btilt);
		float bx3 = bx2 * cos(-brot) - by2 * sin(-brot);
		float by3 = bx2 * sin(-brot) + by2 * cos(-brot);
		float bz3 = bz2;
		float bufDist = sqrt((bx3 - bx4) * (bx3 - bx4) +
			(by3 - by4) * (by3 - by4) + (bz3 - bz4) * (bz3 - bz4));
		bx4 = bx3;
		by4 = by3;
		bz4 = bz3;
		dist += bufDist;
	}
	div = ceil(div * dist / DISTANCE_TARGET);
	// calc dist
	dist = 0.0;
	bx4 = stx2;
	by4 = sty2;
	bz4 = stz2;
	for (int i = 0; i < div; i++)
	{
		float bx1 = (now.x - prev.x) * (i + 1) / div + prev.x;
		float by1 = (now.y - prev.y) * (i + 1) / div + prev.y;
		float bz1 = (now.z - prev.z) * (i + 1) / div + prev.z;
		float brot = (now.c - prev.c) * (i + 1) / div + prev.c;
		float btilt = (now.b - prev.b) * (i + 1) / div + prev.b;
		float bx2 = bx1 * cos(-btilt) + bz1 * sin(-btilt);
		float by2 = by1;
		float bz2 = -bx1 * sin(-btilt) + bz1 * cos(-btilt);
		float bx3 = bx2 * cos(-brot) - by2 * sin(-brot);
		float by3 = bx2 * sin(-brot) + by2 * cos(-brot);
		float bz3 = bz2;
		float bufDist = sqrt((bx3 - bx4) * (bx3 - bx4) +
			(by3 - by4) * (by3 - by4) + (bz3 - bz4) * (bz3 - bz4));
		bx4 = bx3;
		by4 = by3;
		bz4 = bz3;
		dist += bufDist;
	}
	return dist;
}

void CoreXYBC::GetRotationMatrix(float a, float b, float c, float* mat)
{
	float matb[3 * 3];
	matb[0] = cos(-b);
	matb[1] = 0.0;
	matb[2] = sin(-b);
	matb[3] = 0.0;
	matb[4] = 1.0;
	matb[5] = 0.0;
	matb[6] = -sin(-b);
	matb[7] = 0.0;
	matb[8] = cos(-b);
	float matc[3 * 3];
	matc[0] = cos(-c);
	matc[1] = -sin(-c);
	matc[2] = 0.0;
	matc[3] = sin(-c);
	matc[4] = cos(-c);
	matc[5] = 0.0;
	matc[6] = 0.0;
	matc[7] = 0.0;
	matc[8] = 1.0;
	calcMatMat(matc, matb, mat);
}

void CoreXYBC::DisplayStart(CameraParam camPrm, PrintItem item)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	float bufCamera[3] = {
		sin(camPrm.cameraAngleZ) * cos(camPrm.cameraAngleX),
		-sin(camPrm.cameraAngleX),
		cos(camPrm.cameraAngleZ) * cos(camPrm.cameraAngleX) };

	float mat[3 * 3];
	GetRotationMatrix(item.cmdVal[3], item.cmdVal[4], item.cmdVal[5], mat);
	float bufCamera2[3];
	bufCamera2[0] = bufCamera[0];
	bufCamera2[1] = bufCamera[1];
	bufCamera2[2] = bufCamera[2];
	float bufCamera3[3];
	calcMatVec(mat, bufCamera2, bufCamera3);
	float eyePos[3];
	float centerPos[3];
	float upVec[3];
	float bufCenter[3];
	bufCenter[0] = camPrm.targetPosX;
	bufCenter[1] = camPrm.targetPosY;
	bufCenter[2] = camPrm.targetPosZ;
	calcMatVec(mat, bufCenter, centerPos);
	eyePos[0] = bufCamera3[0] * camPrm.cameraDistance + centerPos[0];
	eyePos[1] = bufCamera3[1] * camPrm.cameraDistance + centerPos[1];
	eyePos[2] = bufCamera3[2] * camPrm.cameraDistance + centerPos[2] + item.cmdVal[2];
	upVec[0] = item.headNorm[0];
	upVec[1] = item.headNorm[1];
	upVec[2] = item.headNorm[2];
	gluLookAt(eyePos[0], eyePos[1], eyePos[2], centerPos[0], centerPos[1], centerPos[2], upVec[0], upVec[1], upVec[2]);

	//printf("%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
	//	print[timeCnt].cmdVal[4], print[timeCnt].cmdVal[5],
	//	eyePos[0], eyePos[1], eyePos[2], centerPos[0], centerPos[1], centerPos[2], upVec[0], upVec[1], upVec[2]);
}

CoreXY::CoreXY()
{
}

CoreXY::~CoreXY()
{
}

GcodeItem CoreXY::InverseKinematics4Item(PathItem path, GcodeItem prev)
{
	GcodeItem ret;
	// value set
	ret.x = path.pnt[0];
	ret.y = path.pnt[1];
	ret.z = path.pnt[2];
	ret.a = 0.0;
	ret.b = 0.0;
	ret.c = 0.0;
	if (path.type == PathType::OuterWallStart || PathType::InnerWallStart ||
		PathType::InnerWallStart || PathType::SupportStart || PathType::Saving)
	{
		ret.gORm = true;
		ret.codeVal = 0;
	}
	else
	{
		ret.gORm = true;
		ret.codeVal = 1;
	}
	// distance
	ret.distance = CalcDistance(prev, ret);

	return ret;
}

float* CoreXY::ForwardKinematics(float x, float y, float z, float a, float b, float c)
{
	float ret[3];
	ret[0] = x;
	ret[1] = y;
	ret[2] = z;
	return ret;
}

PathItem CoreXY::ForwardKinematics4Item(GcodeItem gcode)
{
	PathItem ret;
	// pos
	ret.pnt[0] = gcode.x;
	ret.pnt[1] = gcode.y;
	ret.pnt[2] = gcode.z;
	// norm
	ret.norm[0] = 0.0;
	ret.norm[1] = 0.0;
	ret.norm[2] = 1.0;
	// other
	ret.width = gcode.width;
	ret.thick = gcode.thick;
	return ret;
}

float CoreXY::CalcDistance(GcodeItem prev, GcodeItem now)
{
	double buf2 = (now.x - prev.x) * (now.x - prev.x) + (now.y - prev.y) * (now.y - prev.y) + (now.z - prev.z) * (now.z - prev.z);
	double dist = sqrt(buf2);
	return dist;
}

void CoreXY::GetRotationMatrix(float a, float b, float c, float* mat)
{
	mat[0] = 1.0;
	mat[1] = 0.0;
	mat[2] = 0.0;
	mat[3] = 0.0;
	mat[4] = 1.0;
	mat[5] = 0.0;
	mat[6] = 0.0;
	mat[7] = 0.0;
	mat[8] = 1.0;
}

void CoreXY::DisplayStart(CameraParam camPrm, PrintItem item)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	float bufCamera[3] = {
		sin(camPrm.cameraAngleZ) * cos(camPrm.cameraAngleX),
		-sin(camPrm.cameraAngleX),
		cos(camPrm.cameraAngleZ) * cos(camPrm.cameraAngleX) };

	float mat[3 * 3];
	GetRotationMatrix(0.0, 0.0, 1.0, mat);
	float bufCamera2[3];
	bufCamera2[0] = bufCamera[0];
	bufCamera2[1] = bufCamera[1];
	bufCamera2[2] = bufCamera[2];
	float bufCamera3[3];
	calcMatVec(mat, bufCamera2, bufCamera3);
	float eyePos[3];
	float centerPos[3];
	float upVec[3];
	float bufCenter[3];
	bufCenter[0] = camPrm.targetPosX;
	bufCenter[1] = camPrm.targetPosY;
	bufCenter[2] = camPrm.targetPosZ;
	calcMatVec(mat, bufCenter, centerPos);
	eyePos[0] = bufCamera3[0] * camPrm.cameraDistance + centerPos[0];
	eyePos[1] = bufCamera3[1] * camPrm.cameraDistance + centerPos[1];
	eyePos[2] = bufCamera3[2] * camPrm.cameraDistance + centerPos[2] + item.cmdVal[2];
	upVec[0] = item.headNorm[0];
	upVec[1] = item.headNorm[1];
	upVec[2] = item.headNorm[2];
	gluLookAt(eyePos[0], eyePos[1], eyePos[2], centerPos[0], centerPos[1], centerPos[2], upVec[0], upVec[1], upVec[2]);

	//printf("%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
	//	print[timeCnt].cmdVal[4], print[timeCnt].cmdVal[5],
	//	eyePos[0], eyePos[1], eyePos[2], centerPos[0], centerPos[1], centerPos[2], upVec[0], upVec[1], upVec[2]);
}

BedSlingerY::BedSlingerY()
{
}

BedSlingerY::~BedSlingerY()
{
}

GcodeItem BedSlingerY::InverseKinematics4Item(PathItem path, GcodeItem prev)
{
	GcodeItem ret;
	// value set
	ret.x = path.pnt[0];
	ret.y = path.pnt[1];
	ret.z = path.pnt[2];
	ret.a = 0.0;
	ret.b = 0.0;
	ret.c = 0.0;
	if (path.type == PathType::OuterWallStart || PathType::InnerWallStart ||
		PathType::InnerWallStart || PathType::SupportStart || PathType::Saving)
	{
		ret.gORm = true;
		ret.codeVal = 0;
	}
	else
	{
		ret.gORm = true;
		ret.codeVal = 1;
	}
	// distance
	ret.distance = CalcDistance(prev, ret);

	return ret;
}

float* BedSlingerY::ForwardKinematics(float x, float y, float z, float a, float b, float c)
{
	float ret[3];
	ret[0] = x;
	ret[1] = y;
	ret[2] = z;
	return ret;
}

PathItem BedSlingerY::ForwardKinematics4Item(GcodeItem gcode)
{
	PathItem ret;
	// pos
	ret.pnt[0] = gcode.x;
	ret.pnt[1] = gcode.y;
	ret.pnt[2] = gcode.z;
	// norm
	ret.norm[0] = 0.0;
	ret.norm[1] = 0.0;
	ret.norm[2] = 1.0;
	// other
	ret.width = gcode.width;
	ret.thick = gcode.thick;
	return ret;
}

float BedSlingerY::CalcDistance(GcodeItem prev, GcodeItem now)
{
	double buf2 = (now.x - prev.x) * (now.x - prev.x) + (now.y - prev.y) * (now.y - prev.y) + (now.z - prev.z) * (now.z - prev.z);
	double dist = sqrt(buf2);
	return dist;
}

void BedSlingerY::GetRotationMatrix(float a, float b, float c, float* mat)
{
	mat[0] = 1.0;
	mat[1] = 0.0;
	mat[2] = 0.0;
	mat[3] = 0.0;
	mat[4] = 1.0;
	mat[5] = 0.0;
	mat[6] = 0.0;
	mat[7] = 0.0;
	mat[8] = 1.0;
}

void BedSlingerY::DisplayStart(CameraParam camPrm, PrintItem item)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	float bufCamera[3] = {
		sin(camPrm.cameraAngleZ) * cos(camPrm.cameraAngleX),
		-sin(camPrm.cameraAngleX),
		cos(camPrm.cameraAngleZ) * cos(camPrm.cameraAngleX) };

	float mat[3 * 3];
	GetRotationMatrix(0.0, 0.0, 1.0, mat);
	float bufCamera2[3];
	bufCamera2[0] = bufCamera[0];
	bufCamera2[1] = bufCamera[1];
	bufCamera2[2] = bufCamera[2];
	float bufCamera3[3];
	calcMatVec(mat, bufCamera2, bufCamera3);
	float eyePos[3];
	float centerPos[3];
	float upVec[3];
	float bufCenter[3];
	bufCenter[0] = camPrm.targetPosX;
	bufCenter[1] = camPrm.targetPosY;
	bufCenter[2] = camPrm.targetPosZ;
	calcMatVec(mat, bufCenter, centerPos);
	eyePos[0] = bufCamera3[0] * camPrm.cameraDistance + centerPos[0];
	eyePos[1] = bufCamera3[1] * camPrm.cameraDistance + centerPos[1] + item.cmdVal[1];
	eyePos[2] = bufCamera3[2] * camPrm.cameraDistance + centerPos[2];
	upVec[0] = item.headNorm[0];
	upVec[1] = item.headNorm[1];
	upVec[2] = item.headNorm[2];
	gluLookAt(eyePos[0], eyePos[1], eyePos[2], centerPos[0], centerPos[1], centerPos[2], upVec[0], upVec[1], upVec[2]);

	//printf("%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
	//	print[timeCnt].cmdVal[4], print[timeCnt].cmdVal[5],
	//	eyePos[0], eyePos[1], eyePos[2], centerPos[0], centerPos[1], centerPos[2], upVec[0], upVec[1], upVec[2]);
}

Delta::Delta()
{
}

Delta::~Delta()
{
}

GcodeItem Delta::InverseKinematics4Item(PathItem path, GcodeItem prev)
{
	GcodeItem ret;
	// value set
	ret.x = path.pnt[0];
	ret.y = path.pnt[1];
	ret.z = path.pnt[2];
	ret.a = 0.0;
	ret.b = 0.0;
	ret.c = 0.0;
	if (path.type == PathType::OuterWallStart || PathType::InnerWallStart ||
		PathType::InnerWallStart || PathType::SupportStart || PathType::Saving)
	{
		ret.gORm = true;
		ret.codeVal = 0;
	}
	else
	{
		ret.gORm = true;
		ret.codeVal = 1;
	}
	// distance
	ret.distance = CalcDistance(prev, ret);

	return ret;
}

float* Delta::ForwardKinematics(float x, float y, float z, float a, float b, float c)
{
	float ret[3];
	ret[0] = x;
	ret[1] = y;
	ret[2] = z;
	return ret;
}

PathItem Delta::ForwardKinematics4Item(GcodeItem gcode)
{
	PathItem ret;
	// pos
	ret.pnt[0] = gcode.x;
	ret.pnt[1] = gcode.y;
	ret.pnt[2] = gcode.z;
	// norm
	ret.norm[0] = 0.0;
	ret.norm[1] = 0.0;
	ret.norm[2] = 1.0;
	// other
	ret.width = gcode.width;
	ret.thick = gcode.thick;
	return ret;
}

float Delta::CalcDistance(GcodeItem prev, GcodeItem now)
{
	double buf2 = (now.x - prev.x) * (now.x - prev.x) + (now.y - prev.y) * (now.y - prev.y) + (now.z - prev.z) * (now.z - prev.z);
	double dist = sqrt(buf2);
	return dist;
}

void Delta::GetRotationMatrix(float a, float b, float c, float* mat)
{
	mat[0] = 1.0;
	mat[1] = 0.0;
	mat[2] = 0.0;
	mat[3] = 0.0;
	mat[4] = 1.0;
	mat[5] = 0.0;
	mat[6] = 0.0;
	mat[7] = 0.0;
	mat[8] = 1.0;
}

void Delta::DisplayStart(CameraParam camPrm, PrintItem item)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	float bufCamera[3] = {
		sin(camPrm.cameraAngleZ) * cos(camPrm.cameraAngleX),
		-sin(camPrm.cameraAngleX),
		cos(camPrm.cameraAngleZ) * cos(camPrm.cameraAngleX) };

	float mat[3 * 3];
	GetRotationMatrix(0.0, 0.0, 1.0, mat);
	float bufCamera2[3];
	bufCamera2[0] = bufCamera[0];
	bufCamera2[1] = bufCamera[1];
	bufCamera2[2] = bufCamera[2];
	float bufCamera3[3];
	calcMatVec(mat, bufCamera2, bufCamera3);
	float eyePos[3];
	float centerPos[3];
	float upVec[3];
	float bufCenter[3];
	bufCenter[0] = camPrm.targetPosX;
	bufCenter[1] = camPrm.targetPosY;
	bufCenter[2] = camPrm.targetPosZ;
	calcMatVec(mat, bufCenter, centerPos);
	eyePos[0] = bufCamera3[0] * camPrm.cameraDistance + centerPos[0];
	eyePos[1] = bufCamera3[1] * camPrm.cameraDistance + centerPos[1];
	eyePos[2] = bufCamera3[2] * camPrm.cameraDistance + centerPos[2];
	upVec[0] = item.headNorm[0];
	upVec[1] = item.headNorm[1];
	upVec[2] = item.headNorm[2];
	gluLookAt(eyePos[0], eyePos[1], eyePos[2], centerPos[0], centerPos[1], centerPos[2], upVec[0], upVec[1], upVec[2]);

	//printf("%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
	//	print[timeCnt].cmdVal[4], print[timeCnt].cmdVal[5],
	//	eyePos[0], eyePos[1], eyePos[2], centerPos[0], centerPos[1], centerPos[2], upVec[0], upVec[1], upVec[2]);
}