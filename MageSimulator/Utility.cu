#include "Utility.cuh"

void calcMatMat(float ma[3 * 3], float mb[3 * 3], float ret[3 * 3])
{
	ret[3 * 0 + 0] = ma[3 * 0 + 0] * mb[3 * 0 + 0] 
		+ ma[3 * 0 + 1] * mb[3 * 1 + 0] + ma[3 * 0 + 2] * mb[3 * 2 + 0];
	ret[3 * 0 + 1] = ma[3 * 0 + 0] * mb[3 * 0 + 1] 
		+ ma[3 * 0 + 1] * mb[3 * 1 + 1] + ma[3 * 0 + 2] * mb[3 * 2 + 1];
	ret[3 * 0 + 2] = ma[3 * 0 + 0] * mb[3 * 0 + 2] 
		+ ma[3 * 0 + 1] * mb[3 * 1 + 2] + ma[3 * 0 + 2] * mb[3 * 2 + 2];
	ret[3 * 1 + 0] = ma[3 * 1 + 0] * mb[3 * 0 + 0] 
		+ ma[3 * 1 + 1] * mb[3 * 1 + 0] + ma[3 * 1 + 2] * mb[3 * 2 + 0];
	ret[3 * 1 + 1] = ma[3 * 1 + 0] * mb[3 * 0 + 1] 
		+ ma[3 * 1 + 1] * mb[3 * 1 + 1] + ma[3 * 1 + 2] * mb[3 * 2 + 1];
	ret[3 * 1 + 2] = ma[3 * 1 + 0] * mb[3 * 0 + 2]
		+ ma[3 * 1 + 1] * mb[3 * 1 + 2] + ma[3 * 1 + 2] * mb[3 * 2 + 2];
	ret[3 * 2 + 0] = ma[3 * 2 + 0] * mb[3 * 0 + 0] 
		+ ma[3 * 2 + 1] * mb[3 * 1 + 0] + ma[3 * 2 + 2] * mb[3 * 2 + 0];
	ret[3 * 2 + 1] = ma[3 * 2 + 0] * mb[3 * 0 + 1] 
		+ ma[3 * 2 + 1] * mb[3 * 1 + 1] + ma[3 * 2 + 2] * mb[3 * 2 + 1];
	ret[3 * 2 + 2] = ma[3 * 2 + 0] * mb[3 * 0 + 2] 
		+ ma[3 * 2 + 1] * mb[3 * 1 + 2] + ma[3 * 2 + 2] * mb[3 * 2 + 2];
}

void calcMatVec(float m[3 * 3], float v[3], float ret[3])
{
	ret[0] = m[3 * 0 + 0] * v[0] + m[3 * 0 + 1] * v[1] + m[3 * 0 + 2] * v[2];
	ret[1] = m[3 * 1 + 0] * v[0] + m[3 * 1 + 1] * v[1] + m[3 * 1 + 2] * v[2];
	ret[2] = m[3 * 2 + 0] * v[0] + m[3 * 2 + 1] * v[1] + m[3 * 2 + 2] * v[2];
}

void calcRot2Quat(float rx, float ry, float rz,
	float* nx, float* ny, float* nz, float* angle)
{
	float cx = cos(rx / 2.0);
	float sx = sin(rx / 2.0);
	float cy = cos(ry / 2.0);
	float sy = sin(ry / 2.0);
	float cz = cos(rz / 2.0);
	float sz = sin(rz / 2.0);
	float q[4];
	q[0] = cx * sy * sz + sx * cy * cz;
	q[1] = -sx * cy * sz + cx * sy * cz;
	q[2] = cx * cy * sz - sx * sy * cz;
	q[3] = sx * sy * sz + cx * cy * cz;
	*angle = acos(q[3]) * 2.0;
	if (sin(*angle / 2.0) == 0.0)
	{
		*nx = 0.0;
		*ny = 0.0;
		*nz = 1.0;
	}
	else
	{
		*nx = q[0] / sin(*angle / 2.0);
		*ny = q[1] / sin(*angle / 2.0);
		*nz = q[2] / sin(*angle / 2.0);
		float nlen = sqrt(*nx * *nx + *ny * *ny + *nz * *nz);
		if (nlen == 0.0)
		{
			*nx = 0.0;
			*ny = 0.0;
			*nz = 1.0;
		}
		else
		{
			*nx = *nx / nlen;
			*ny = *ny / nlen;
			*nz = *nz / nlen;
		}
	}
}

void calcQuat2RotYxz(const float nx, const float ny, const float nz,
	const float angle, float m[3 * 3])
{
	float qx = nx * sin(angle / 2.0);
	float qy = ny * sin(angle / 2.0);
	float qz = nz * sin(angle / 2.0);
	float qw = cos(angle / 2.0);
	m[3 * 0 + 0] = 2.0 * qw * qw + 2.0 * qx * qx - 1.0;
	m[3 * 0 + 1] = 2.0 * qx * qy - 2.0 * qz * qw;
	m[3 * 0 + 2] = 2.0 * qx * qz + 2.0 * qy * qw;
	m[3 * 1 + 0] = 2.0 * qx * qy + 2.0 * qz * qw;
	m[3 * 1 + 1] = 2.0 * qw * qw + 2.0 * qy * qy - 1.0;
	m[3 * 1 + 2] = 2.0 * qy * qz - 2.0 * qx * qw;
	m[3 * 2 + 0] = 2.0 * qx * qz - 2.0 * qy * qw;
	m[3 * 2 + 1] = 2.0 * qy * qz + 2.0 * qx * qw;
	m[3 * 2 + 2] = 2.0 * qw * qw + 2.0 * qz * qz - 1.0;
}

void calcQuat_byVec2Vec(float stx, float sty, float stz, float edx, float edy, float edz,
	float* nx, float* ny, float* nz, float* angle)
{
	float stlen = sqrt(stx * stx + sty * sty + stz * stz);
	if (stlen == 0.0) stlen = 1.0;
	stx = stx / stlen;
	sty = sty / stlen;
	stz = stz / stlen;
	float edlen = sqrt(edx * edx + edy * edy + edz * edz);
	if (edlen == 0.0) edlen = 1.0;
	edx = edx / edlen;
	edy = edy / edlen;
	edz = edz / edlen;
	// norm
	*nx = sty * edz - stz * edy;
	*ny = stz * edx - stx * edz;
	*nz = stx * edy - sty * edx;
	*angle = acos(stx * edx + sty * edy + stz * edz);
	float nlen = sqrt(*nx * *nx + *ny * *ny + *nz * *nz);
	if (nlen == 0.0)
	{
		*nx = 0.0;
		*ny = 0.0;
		*nz = 1.0;
	}
	else
	{
		*nx = *nx / nlen;
		*ny = *ny / nlen;
		*nz = *nz / nlen;
	}
	// check
	float vec[3] = { stx, sty, stz };
	float mat1[3 * 3];
	calcQuat2RotYxz(*nx, *ny, *nz, *angle, mat1);
	float ret1[3];
	calcMatVec(mat1, vec, ret1);
	float chk1 = edx * ret1[0] + edy * ret1[1] + edz * ret1[2];
	(chk1 > 1.0) ? (chk1 = 1.0) : ((chk1 < -1.0) ? (chk1 = -1.0) : (chk1 = chk1));
	chk1 = acos(chk1);
	float mat2[3 * 3];
	calcQuat2RotYxz(-*nx, -*ny, -*nz, *angle, mat2);
	float ret2[3];
	calcMatVec(mat2, vec, ret2);
	float chk2 = edx * ret2[0] + edy * ret2[1] + edz * ret2[2];
	(chk2 > 1.0) ? (chk2 = 1.0) : ((chk2 < -1.0) ? (chk2 = -1.0) : (chk2 = chk2));
	chk2 = acos(chk2);
	if (chk1 > chk2)
	{
		*nx = -*nx;
		*ny = -*ny;
		*nz = -*nz;
	}
}

__device__ __host__ void HSVtoRGB(float H, float S, float V, float& R, float& G, float& B) {
	float C = V * S;
	float X = C * (1 - fabs(fmod(H / 60.0, 2) - 1));
	float m = V - C;

	float r = 0, g = 0, b = 0;

	if (0 <= H && H < 60) {
		r = C; g = X; b = 0;
	}
	else if (60 <= H && H < 120) {
		r = X; g = C; b = 0;
	}
	else if (120 <= H && H < 180) {
		r = 0; g = C; b = X;
	}
	else if (180 <= H && H < 240) {
		r = 0; g = X; b = C;
	}
	else if (240 <= H && H < 300) {
		r = X; g = 0; b = C;
	}
	else if (300 <= H && H < 360) {
		r = C; g = 0; b = X;
	}

	// 最終的なRGB値の計算
	R = (r + m);
	G = (g + m);
	B = (b + m);
}

Mesh* createMesh2Bbox(std::vector<Mesh> org, int id)
{
	float xmin = FLT_MAX;
	float xmax = FLT_MIN;
	float ymin = FLT_MAX;
	float ymax = FLT_MIN;
	float zmin = FLT_MAX;
	float zmax = FLT_MIN;
	for (int i = 0; i < org.size(); i++)
	{
		if (xmin > org[i].pnt0[0]) xmin = org[i].pnt0[0];
		if (xmin > org[i].pnt1[0]) xmin = org[i].pnt1[0];
		if (xmin > org[i].pnt2[0]) xmin = org[i].pnt2[0];
		if (xmax < org[i].pnt0[0]) xmax = org[i].pnt0[0];
		if (xmax < org[i].pnt1[0]) xmax = org[i].pnt1[0];
		if (xmax < org[i].pnt2[0]) xmax = org[i].pnt2[0];

		if (ymin > org[i].pnt0[1]) ymin = org[i].pnt0[1];
		if (ymin > org[i].pnt1[1]) ymin = org[i].pnt1[1];
		if (ymin > org[i].pnt2[1]) ymin = org[i].pnt2[1];
		if (ymax < org[i].pnt0[1]) ymax = org[i].pnt0[1];
		if (ymax < org[i].pnt1[1]) ymax = org[i].pnt1[1];
		if (ymax < org[i].pnt2[1]) ymax = org[i].pnt2[1];

		if (zmin > org[i].pnt0[2]) zmin = org[i].pnt0[2];
		if (zmin > org[i].pnt1[2]) zmin = org[i].pnt1[2];
		if (zmin > org[i].pnt2[2]) zmin = org[i].pnt2[2];
		if (zmax < org[i].pnt0[2]) zmax = org[i].pnt0[2];
		if (zmax < org[i].pnt1[2]) zmax = org[i].pnt1[2];
		if (zmax < org[i].pnt2[2]) zmax = org[i].pnt2[2];
	}
	Mesh* meshs = new Mesh[12];
	float basePnt[8][3] = {
		{xmin, ymax, zmin },
		{xmax, ymax, zmin },
		{xmax, ymax, zmax },
		{xmin, ymax, zmax },
		{xmin, ymin, zmin },
		{xmin, ymin, zmax },
		{xmax, ymin, zmax },
		{xmax, ymin, zmin }
	};
	int baseMesh[12][3] = {
		{0, 1, 2},
		{4, 5, 6},
		{0, 3, 5},
		{3, 2, 6},
		{2, 1, 7},
		{1, 7, 4},
		{0, 2, 3},
		{4, 6, 7},
		{0, 5, 4},
		{3, 6, 5},
		{2, 7, 6},
		{1, 4, 0}
	};
	for (int j = 0; j < 12; j++)
	{
		Mesh buf;
		buf.pnt0[0] = basePnt[baseMesh[j][0]][0];
		buf.pnt0[1] = basePnt[baseMesh[j][0]][1];
		buf.pnt0[2] = basePnt[baseMesh[j][0]][2];
		buf.pnt1[0] = basePnt[baseMesh[j][1]][0];
		buf.pnt1[1] = basePnt[baseMesh[j][1]][1];
		buf.pnt1[2] = basePnt[baseMesh[j][1]][2];
		buf.pnt2[0] = basePnt[baseMesh[j][2]][0];
		buf.pnt2[1] = basePnt[baseMesh[j][2]][1];
		buf.pnt2[2] = basePnt[baseMesh[j][2]][2];
		buf.id = id;
		meshs[j] = buf;
	}
	return meshs;
}
