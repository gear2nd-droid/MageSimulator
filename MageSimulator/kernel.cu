// install nupengl.core by nuget

#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <GL/glew.h>

#include "Structs.cuh"
#include <stdio.h>
#include "ToolPath.cuh"
#include "PrintObject.cuh"
#include "MachineObject.cuh"
#include "CollisionCheck.cuh"
#include "SimpleGL.cuh"
#include <vector>
#include <set>
#include <utility>

#define _USE_MATH_DEFINES
#include <math.h>
#include <time.h>
#include <thread>

// need install "vcpkg install tinyxml2"
#include <tinyxml2.h>
#include <omp.h>

PathItem* toolpath;
PrintItem* print;
std::vector<std::vector<PrintItem*>> largePrint;
std::vector<Ray*> largeRay;
std::vector<Mesh*> largeMesh;
int toolpathFullCnt;
int printFullCnt;
std::vector<int> printHeader;
bool isloop = true;
int timeCnt = 0;
Kinematics* kin;
std::vector<CollisionObject*> arrBodies;
std::vector<CollisionObject*> arrBeds;
bool enableCheckCollsion = false;
bool enableMultiThread = false;
std::vector<CollisionSet>* resultCol;

GLuint pbo;
cudaGraphicsResource* dev_resource;

std::vector<CollisionSet> checkTime(int time, bool isdraw)
{
	clock_t time1 = clock();
	std::vector<CollisionSet> retcol = std::vector<CollisionSet>();
	// large
	std::vector<Ray> largeRay = std::vector<Ray>();
	std::vector<std::vector<Mesh>> bodyMeshBuffer = std::vector<std::vector<Mesh>>();
	bool** targetGroup = new bool* [arrBodies.size()];
	for (int m = 0; m < arrBodies.size(); m++)
	{
		targetGroup[m] = new bool[largeMesh.size()];
		for (int i = 0; i < largeMesh.size(); i++)
		{
			targetGroup[m][i] = false;
		}
		std::vector<Mesh> arrBodyMesh = arrBodies[m]->move(print[time], kin, isdraw);
		bodyMeshBuffer.push_back(arrBodyMesh);
		for (int n = 0; n < arrBodyMesh.size(); n++)
		{
			Ray buf0;
			buf0.org[0] = arrBodyMesh[n].pnt0[0];
			buf0.org[1] = arrBodyMesh[n].pnt0[1];
			buf0.org[2] = arrBodyMesh[n].pnt0[2];
			buf0.tar[0] = arrBodyMesh[n].pnt1[0];
			buf0.tar[1] = arrBodyMesh[n].pnt1[1];
			buf0.tar[2] = arrBodyMesh[n].pnt1[2];
			buf0.id = m;
			largeRay.push_back(buf0);
			Ray buf1;
			buf1.org[0] = arrBodyMesh[n].pnt1[0];
			buf1.org[1] = arrBodyMesh[n].pnt1[1];
			buf1.org[2] = arrBodyMesh[n].pnt1[2];
			buf1.tar[0] = arrBodyMesh[n].pnt2[0];
			buf1.tar[1] = arrBodyMesh[n].pnt2[1];
			buf1.tar[2] = arrBodyMesh[n].pnt2[2];
			buf1.id = m;
			largeRay.push_back(buf1);
			Ray buf2;
			buf2.org[0] = arrBodyMesh[n].pnt2[0];
			buf2.org[1] = arrBodyMesh[n].pnt2[1];
			buf2.org[2] = arrBodyMesh[n].pnt2[2];
			buf2.tar[0] = arrBodyMesh[n].pnt0[0];
			buf2.tar[1] = arrBodyMesh[n].pnt0[1];
			buf2.tar[2] = arrBodyMesh[n].pnt0[2];
			buf2.id = m;
			largeRay.push_back(buf2);
		}
	}
	clock_t time7 = clock();
	std::vector<Mesh> largeTargetMesh = std::vector<Mesh>();
	for (int j = 0; j < largeMesh.size(); j++)
	{
		for (int k = 0; k < 12; k++)
		{
			largeTargetMesh.push_back(largeMesh[j][k]);
		}
	}
	//std::vector<Mesh>* bedMeshBuffer = new std::vector<Mesh>[largeRay.size()];
	std::vector<Mesh>* bedMeshBuffer = new std::vector<Mesh>[arrBeds.size()];
	for (int j = 0; j < arrBeds.size(); j++)
	{
		std::vector<Mesh> arrBedMesh = arrBeds[j]->move(print[time], kin, isdraw);
		bedMeshBuffer[j] = arrBedMesh;
		Mesh* bboxMesh = createMesh2Bbox(arrBedMesh, largeMesh.size() + j);
		for (int k = 0; k < 12; k++)
		{
			largeTargetMesh.push_back(bboxMesh[k]);
		}
	}
	Mesh* bufLargeMesh;
	bufLargeMesh = (Mesh*)malloc(sizeof(Mesh) * largeTargetMesh.size() * largeRay.size());
	Ray* bufLargeRay;
	bufLargeRay = (Ray*)malloc(sizeof(Ray) * largeTargetMesh.size() * largeRay.size());
	clock_t time9 = clock();
	createLargeArrays(largeTargetMesh, largeRay, bufLargeMesh, bufLargeRay);

	clock_t time2 = clock();
	CollisionSet* largeCol = (CollisionSet*)malloc(sizeof(CollisionSet) * largeTargetMesh.size() * largeRay.size());
	clock_t time3 = clock();
	checkCollisions(largeTargetMesh.size() * largeRay.size(), bufLargeMesh, bufLargeRay, largeCol);
	clock_t time8 = clock();
	std::vector<std::pair<int, int>> pairBodyGroup = std::vector<std::pair<int, int>>();
	for (int i = 0; i < largeTargetMesh.size() * largeRay.size(); i++)
	{
		if (largeCol[i].colFlag)
		{
			std::pair<int, int > pair = std::pair<int, int>();
			pair.first = largeCol[i].rayId;
			pair.second = largeCol[i].meshId;
			pairBodyGroup.push_back(pair);
		}
	}
	std::sort(pairBodyGroup.begin(), pairBodyGroup.end());
	auto last = std::unique(pairBodyGroup.begin(), pairBodyGroup.end());
	pairBodyGroup.erase(last, pairBodyGroup.end());
	// small
	std::vector<Mesh> smallMesh = std::vector<Mesh>();
	std::vector<Ray> smallRay = std::vector<Ray>();
	for (int i = 0; i < pairBodyGroup.size(); i++)
	{
		int body = pairBodyGroup[i].first;
		int group = pairBodyGroup[i].second;
		for (int j = 0; j < bodyMeshBuffer[body].size(); j++)
		{
			if (group < largeMesh.size())
			{
				for (int k = 0; k < largePrint[group].size(); k++)
				{
					if (largePrint[group][k]->time < time)
					{
						for (int m = 0; m < 12; m++)
						{
							smallMesh.push_back(bodyMeshBuffer[body][j]);
							smallRay.push_back(largePrint[group][k]->ray[m]);
						}
					}
				}
			}
			else
			{
				int idx = group - largeMesh.size();
				for (int m = 0; m < bedMeshBuffer[idx].size(); m++)
				{
					Ray buf0;
					buf0.org[0] = bedMeshBuffer[idx][m].pnt0[0];
					buf0.org[1] = bedMeshBuffer[idx][m].pnt0[1];
					buf0.org[2] = bedMeshBuffer[idx][m].pnt0[2];
					buf0.tar[0] = bedMeshBuffer[idx][m].pnt1[0];
					buf0.tar[1] = bedMeshBuffer[idx][m].pnt1[1];
					buf0.tar[2] = bedMeshBuffer[idx][m].pnt1[2];
					buf0.id = group;
					smallMesh.push_back(bodyMeshBuffer[body][j]);
					smallRay.push_back(buf0);
					Ray buf1;
					buf1.org[0] = bedMeshBuffer[idx][m].pnt1[0];
					buf1.org[1] = bedMeshBuffer[idx][m].pnt1[1];
					buf1.org[2] = bedMeshBuffer[idx][m].pnt1[2];
					buf1.tar[0] = bedMeshBuffer[idx][m].pnt2[0];
					buf1.tar[1] = bedMeshBuffer[idx][m].pnt2[1];
					buf1.tar[2] = bedMeshBuffer[idx][m].pnt2[2];
					buf1.id = group;
					smallMesh.push_back(bodyMeshBuffer[body][j]);
					smallRay.push_back(buf1);
					Ray buf2;
					buf2.org[0] = bedMeshBuffer[idx][m].pnt2[0];
					buf2.org[1] = bedMeshBuffer[idx][m].pnt2[1];
					buf2.org[2] = bedMeshBuffer[idx][m].pnt2[2];
					buf2.tar[0] = bedMeshBuffer[idx][m].pnt0[0];
					buf2.tar[1] = bedMeshBuffer[idx][m].pnt0[1];
					buf2.tar[2] = bedMeshBuffer[idx][m].pnt0[2];
					buf2.id = group;
					smallMesh.push_back(bodyMeshBuffer[body][j]);
					smallRay.push_back(buf2);
				}
			}
		}
	}
	Mesh* bufSmallMesh;
	bufSmallMesh = (Mesh*)malloc(sizeof(Mesh) * smallMesh.size());
	Ray* bufSmallRay;
	bufSmallRay = (Ray*)malloc(sizeof(Ray) * smallMesh.size());
	for (int i = 0; i < smallMesh.size(); i++)
	{
		bufSmallMesh[i] = smallMesh[i];
		bufSmallRay[i] = smallRay[i];
	}
	clock_t time4 = clock();
	CollisionSet* smallCol = (CollisionSet*)malloc(sizeof(CollisionSet) * smallMesh.size());
	clock_t time5 = clock();
	checkCollisions(smallMesh.size(), bufSmallMesh, bufSmallRay, smallCol);
	for (int i = 0; i < smallMesh.size(); i++)
	{
		if (smallCol[i].colFlag)
		{
			retcol.push_back(smallCol[i]);
		}
	}

	// memory clear
	free(bufLargeMesh);
	free(bufLargeRay);
	free(largeCol);
	largeRay.clear();
	pairBodyGroup.clear();
	smallMesh.clear();
	smallRay.clear();
	bodyMeshBuffer.clear();
	free(bufSmallMesh);
	free(bufSmallRay);
	free(smallCol);
	clock_t time6 = clock();
	float time17 = static_cast<float>(time7 - time1) / CLOCKS_PER_SEC * 1000.0;
	float time79 = static_cast<float>(time9 - time7) / CLOCKS_PER_SEC * 1000.0;
	float time92 = static_cast<float>(time2 - time9) / CLOCKS_PER_SEC * 1000.0;
	float time23 = static_cast<float>(time3 - time2) / CLOCKS_PER_SEC * 1000.0;
	float time38 = static_cast<float>(time8 - time3) / CLOCKS_PER_SEC * 1000.0;
	float time84 = static_cast<float>(time4 - time8) / CLOCKS_PER_SEC * 1000.0;
	float time45 = static_cast<float>(time5 - time4) / CLOCKS_PER_SEC * 1000.0;
	float time56 = static_cast<float>(time6 - time5) / CLOCKS_PER_SEC * 1000.0;
	//printf("%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n", time17, time79, time92, time23, time38, time84, time45, time56);

	return retcol;
}

void sampling()
{
	if (isloop)
	{
		// drawBox
		float time_gl = 0.0;
		if (timeCnt % 10 == 0)
		{
			extern CameraParam cameraParam;
			kin->DisplayStart(cameraParam, print[timeCnt]);
			drawCoordinate(50.0, print[timeCnt], kin);
			clock_t start_gl = clock();
			for (int i = 0; i < timeCnt; i++)
			{
				if (print[i].type == PrintType::OuterWall || print[i].type == PrintType::Infill || print[i].type == PrintType::Print)
				{
					float h = i / (float)printFullCnt * 360.0;
					float faceR, faceG, faceB, edgeR, edgeG, edgeB;
					HSVtoRGB(h, 1.0, 1.0, faceR, faceG, faceB);
					HSVtoRGB(h, 0.5, 1.0, edgeR, edgeG, edgeB);
					drawPrintItem_box(print[i], faceR, faceG, faceB, edgeR, edgeG, edgeB);
				}
				else
				{
					drawPrintItem_move(print[i], 1.0, 1.0, 1.0);
				}
			}
			clock_t end_gl = clock();
			time_gl = static_cast<float>(end_gl - start_gl) / CLOCKS_PER_SEC * 1000.0;
		}

		// time 
		clock_t start_cuda = clock();
		// collision check
		std::vector<CollisionSet> colset;
		if (enableCheckCollsion)
		{
			colset = checkTime(timeCnt, true);
			for (int i = 0; i < colset.size(); i++)
			{
				glColor3d(0.5, 0.5, 0.5);
				glPushMatrix();
				glTranslatef(colset[i].pnt[0], colset[i].pnt[1], colset[i].pnt[2]);
				glutSolidSphere(1.0, 10, 10);
				glPopMatrix();
			}
		}
		else
		{
			for (int m = 0; m < arrBodies.size(); m++)
			{
				std::vector<Mesh> arrBodyMesh = arrBodies[m]->move(print[timeCnt], kin, true);
			}
			for (int j = 0; j < arrBeds.size(); j++)
			{
				std::vector<Mesh> arrBedMesh = arrBeds[j]->move(print[timeCnt], kin, true);
			}
		}

		// time
		clock_t end_cuda = clock();
		float time_cuda = static_cast<float>(end_cuda - start_cuda) / CLOCKS_PER_SEC * 1000.0;
		if (enableCheckCollsion)
		{
			printf("time:%d,line:%d,block:%d,layer:%d,collision:%d\n",
				timeCnt, print[timeCnt].lineIdx, print[timeCnt].blockIdx, print[timeCnt].layerIdx, colset.size());
		}
		else
		{
			printf("time:%d,line:%d,block:%d,layer:%d\n",
				timeCnt, print[timeCnt].lineIdx, print[timeCnt].blockIdx, print[timeCnt].layerIdx);
		}

		// draw and next time
		if (timeCnt % 10 == 0)
		{
			glutSwapBuffers();
		}
		timeCnt++;
		if (timeCnt >= printFullCnt) timeCnt = printFullCnt - 1;
	}
	else
	{
		extern CameraParam cameraParam;
		kin->DisplayStart(cameraParam, print[timeCnt]);
		drawCoordinate(50.0, print[timeCnt], kin);
		for (int i = 0; i < timeCnt; i++)
		{
			if (print[i].type == PrintType::OuterWall || print[i].type == PrintType::Infill || print[i].type == PrintType::Print)
			{
				float h = i / (float)printFullCnt * 360.0;
				float faceR, faceG, faceB, edgeR, edgeG, edgeB;
				HSVtoRGB(h, 1.0, 1.0, faceR, faceG, faceB);
				HSVtoRGB(h, 0.5, 1.0, edgeR, edgeG, edgeB);
				drawPrintItem_box(print[i], faceR, faceG, faceB, edgeR, edgeG, edgeB);
			}
			else
			{
				drawPrintItem_move(print[i], 1.0, 1.0, 1.0);
			}
		}

		for (int m = 0; m < arrBodies.size(); m++)
		{
			std::vector<Mesh> arrBodyMesh = arrBodies[m]->move(print[timeCnt], kin, true);
		}
		for (int j = 0; j < arrBeds.size(); j++)
		{
			std::vector<Mesh> arrBedMesh = arrBeds[j]->move(print[timeCnt], kin, true);
		}
		glutSwapBuffers();
	}
}

int main(int argc, char** argv)
{
	// read param
	// 1:gcode
	//char* filepath = "C:\\Develop\\Slicer\\slicer_models\\output_viewer.gcode";
	char* filepath = argv[1];
	printf("GcodeFile:%s\n", filepath);
	// 2:machine
	char* machinefile = argv[2];
	printf("MachineFile:%s\n", machinefile);
	char* strCol = argv[3];
	enableCheckCollsion = false;
	if (strcmp(strCol, "true") == 0)
	{
		enableCheckCollsion = true;
	}
	printf("enableCheckCollision:%s\n", (enableCheckCollsion ? "true" : "false"));
	char* strMulti = argv[4];
	enableMultiThread = false;
	if (strcmp(strMulti, "true") == 0)
	{
		enableMultiThread = true;
	}
	printf("enableMultiThread:%s\n", (enableMultiThread ? "true" : "false"));

	// read machine xml
	tinyxml2::XMLDocument doc;
	tinyxml2::XMLError err = doc.LoadFile(machinefile);
	tinyxml2::XMLElement* xmlRoot = doc.FirstChildElement("Machine");
	tinyxml2::XMLElement* xmlKinematics = xmlRoot->FirstChildElement("Kinematics");
	printf("Kinematics:%s", xmlKinematics->GetText());
	if (strcmp(xmlKinematics->GetText(), "CoreXY-BC") == 0)
	{
		kin = new CoreXYBC();
	}
	else if (strcmp(xmlKinematics->GetText(), "CoreXY") == 0)
	{
		kin = new CoreXY();
	}
	else if (strcmp(xmlKinematics->GetText(), "BedSlingerY") == 0)
	{
		kin = new BedSlingerY();
	}
	else if (strcmp(xmlKinematics->GetText(), "Delta") == 0)
	{
		kin = new Delta();
	}
	else
	{
		printf("Unknown Kinematics has been selected.");
		exit(-1);
	}
	arrBodies = std::vector<CollisionObject*>();
	arrBeds = std::vector<CollisionObject*>();
	// head
	tinyxml2::XMLElement* xmlHeads = xmlRoot->FirstChildElement("Heads");
	if (xmlHeads)
	{
		tinyxml2::XMLElement* xmlHead = xmlHeads->FirstChildElement("Head");
		while (xmlHead)
		{
			tinyxml2::XMLElement* xmlType = xmlHead->FirstChildElement("Type");
			if (strcmp(xmlType->GetText(), "Cylinder") == 0)
			{
				CollisionCylinder* cylinder = new CollisionCylinder(MachineType::Head, xmlHead);
				arrBodies.push_back(cylinder);
			}
			else if (strcmp(xmlType->GetText(), "Box") == 0)
			{
				CollisionBox* box = new CollisionBox(MachineType::Head, xmlHead);
				arrBodies.push_back(box);
			}
			xmlHead = xmlHead->NextSiblingElement("Head");
		}
	}
	// xgantry
	tinyxml2::XMLElement* xmlXGantrys = xmlRoot->FirstChildElement("XGantrys");
	if (xmlXGantrys)
	{
		tinyxml2::XMLElement* xmlXGantry = xmlXGantrys->FirstChildElement("XGantry");
		while (xmlXGantry)
		{
			tinyxml2::XMLElement* xmlType = xmlXGantry->FirstChildElement("Type");
			if (strcmp(xmlType->GetText(), "Cylinder") == 0)
			{
				CollisionCylinder* cylinder = new CollisionCylinder(MachineType::XGantry, xmlXGantry);
				arrBodies.push_back(cylinder);
			}
			else if (strcmp(xmlType->GetText(), "Box") == 0)
			{
				CollisionBox* box = new CollisionBox(MachineType::XGantry, xmlXGantry);
				arrBodies.push_back(box);
			}
			xmlXGantry = xmlXGantry->NextSiblingElement("XGantry");
		}
	}
	// ygantry
	tinyxml2::XMLElement* xmlYGantrys = xmlRoot->FirstChildElement("YGantrys");
	if (xmlYGantrys)
	{
		tinyxml2::XMLElement* xmlYGantry = xmlYGantrys->FirstChildElement("YGantry");
		while (xmlYGantry)
		{
			tinyxml2::XMLElement* xmlType = xmlYGantry->FirstChildElement("Type");
			if (strcmp(xmlType->GetText(), "Cylinder") == 0)
			{
				CollisionCylinder* cylinder = new CollisionCylinder(MachineType::YGantry, xmlYGantry);
				arrBodies.push_back(cylinder);
			}
			else if (strcmp(xmlType->GetText(), "Box") == 0)
			{
				CollisionBox* box = new CollisionBox(MachineType::YGantry, xmlYGantry);
				arrBodies.push_back(box);
			}
			xmlYGantry = xmlYGantry->NextSiblingElement("YGantry");
		}
	}
	// bed
	tinyxml2::XMLElement* xmlBeds = xmlRoot->FirstChildElement("Beds");
	if (xmlBeds)
	{
		tinyxml2::XMLElement* xmlBed = xmlBeds->FirstChildElement("Bed");
		while (xmlBed)
		{
			tinyxml2::XMLElement* xmlType = xmlBed->FirstChildElement("Type");
			if (strcmp(xmlType->GetText(), "Cylinder") == 0)
			{
				CollisionCylinder* cylinder = new CollisionCylinder(MachineType::Bed, xmlBed);
				arrBeds.push_back(cylinder);
			}
			else if (strcmp(xmlType->GetText(), "Box") == 0)
			{
				CollisionBox* box = new CollisionBox(MachineType::Bed, xmlBed);
				arrBeds.push_back(box);
			}
			xmlBed = xmlBed->NextSiblingElement("Bed");
		}
	}

	// read gcode
	std::vector<GcodeItem> bufgcode = readGcode(filepath);
	int gcodeCnt = bufgcode.size();
	std::vector<PathItem> bufpath = convertGcode2Path(kin, bufgcode);

  // read csv
	toolpathFullCnt = bufpath.size();
  toolpath = (PathItem*)malloc(sizeof(PathItem) * toolpathFullCnt);
  for (int i = 0; i < toolpathFullCnt; i++)
  {
    toolpath[i] = bufpath[i];
  }
	printFullCnt = toolpathFullCnt - 1;
  print = (PrintItem*)malloc(sizeof(PrintItem) * printFullCnt);
  cudaError_t cudaStatus = calcPrintItems(toolpathFullCnt, toolpath, print);
  if (cudaStatus != cudaSuccess) {
    fprintf(stderr, "addWithCuda failed!");
    return 1;
  }
	printf("read file end, step count:%d\n", printFullCnt);

	// header check
	int preBlock = -1;
	int preLayer = -1;
	printHeader = std::vector<int>();
	for (int i = 0; i < printFullCnt; i++)
	{
		int block = print[i].blockIdx;
		int layer = print[i].layerIdx;
		if (preBlock != block || preLayer != layer)
		{
			printHeader.push_back(i);
		}
		preBlock = block;
		preLayer = layer;
	}

	std::vector<PrintItem*> bufPrint;
	for (int i = 0; i < printFullCnt; i++)
	{
		bufPrint.push_back(&print[i]);
	}
	largePrint = std::vector<std::vector<PrintItem*>>();
	largeRay = std::vector<Ray*>();
	largeMesh = std::vector<Mesh*>();
	constructBVH(bufPrint, &largePrint, &largeRay, &largeMesh);

	if (enableMultiThread)
	{
		// time split
		int numThread = std::thread::hardware_concurrency();
		//int numThread = 1;
		printf("NumberThread:%d\n", numThread);
		printf("Start multi thread\n");
		
		std::vector<int> divTimeFirst = std::vector<int>();
		std::vector<int> divTimeLast = std::vector<int>();
		divTimeFirst.push_back(0);
		for (int i = 0; i < numThread; i++)
		{
			float val = (float)printFullCnt * (float)printFullCnt / (float)numThread + (float)divTimeFirst[i] * (float)divTimeFirst[i];
			int bufval = sqrt(val);
			if (i == numThread - 1)
			{
				bufval = printFullCnt - 1;
			}
			divTimeLast.push_back(bufval);
			printf("%d,%d,%d\n", i, divTimeFirst[i], divTimeLast[i] - divTimeFirst[i]);
			divTimeFirst.push_back(bufval);
		}
		resultCol = new std::vector<CollisionSet>[printFullCnt];

#pragma omp parallel for schedule(static)
		for (int i = 0; i < numThread; i++)
		{
			for (int j = divTimeFirst[i]; j < divTimeLast[i]; j++)
			{
				clock_t start_cuda = clock();
				std::vector<CollisionSet> colset = checkTime(j, false);
				resultCol[j] = colset;
				clock_t end_cuda = clock();
				float time_cuda = static_cast<float>(end_cuda - start_cuda) / CLOCKS_PER_SEC * 1000.0;
#pragma omp critical
				{
					printf("%d,%d,%d,%lf\n", i, j, colset.size(), time_cuda);
				}
			}
		}
#pragma omp barrier
		
		printf("End multi thread\n");
	}
	else
	{
		// opengl
		init(argc, argv);
	}

  return 0;
}
