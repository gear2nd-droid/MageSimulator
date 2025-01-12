#pragma once
#ifndef TOOL_PATH
#define TOOL_PATH

#include "Structs.cuh"
#include <vector>
#include <string>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include "ToolPath.cuh"
#include "Kinematics.cuh"

#define CONVERT_PITCH 0.5


std::vector<std::string> split(const std::string& str, char delimiter);
std::vector<PathItem> readToolPath(char* filename);
std::vector<GcodeItem> readGcode(char* filename);
std::vector<PathItem> convertGcode2Path(Kinematics* kin, std::vector<GcodeItem> gcode);

#endif