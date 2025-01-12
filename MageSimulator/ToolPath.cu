#include "ToolPath.cuh"
#include <regex>
#include <string>


std::vector<std::string> split(const std::string& str, char delimiter)
{
	std::vector<std::string> tokens;
	std::string token;
	std::istringstream tokenStream(str);

	while (std::getline(tokenStream, token, delimiter)) {
		tokens.push_back(token);
	}

	return tokens;
}

std::vector<PathItem> readToolPath(char* filename)
{
	std::vector<PathItem> bufPath;
	std::ifstream file(filename);
	if (!file.is_open())
	{
		printf("ERROR: failed to open file");
		return bufPath;
	}

	// read file
	std::string line;
	while (std::getline(file, line)) {
		std::vector<std::string> token = split(line, ',');
		PathItem item = PathItem();
		item.type = (PathType)std::stoi(token[0]);
		item.pnt[0] = std::stof(token[1]);
		item.pnt[1] = std::stof(token[2]);
		item.pnt[2] = std::stof(token[3]);
		item.norm[0] = std::stof(token[4]);
		item.norm[1] = std::stof(token[5]);
		item.norm[2] = std::stof(token[6]);
		item.thick = std::stof(token[7]);
		item.width = std::stof(token[8]);
		item.blockIdx = std::stoi(token[9]);
		item.layerIdx = std::stoi(token[10]);
		item.loopIdx = std::stoi(token[11]);
		item.curveIdx = std::stoi(token[12]);
		bufPath.push_back(item);
	}
	file.close();

	return bufPath;
}

std::vector<GcodeItem> readGcode(char* filename)
{
	std::vector<GcodeItem> bufGcode;
	std::ifstream file(filename);
	if (!file.is_open())
	{
		printf("ERROR: failed to open file");
		return bufGcode;
	}
	// read file
	std::string line;
	float preX = 0.0;
	float preY = 0.0;
	float preZ = 0.0;
	float preA = 0.0;
	float preB = 0.0;
	float preC = 0.0;
	float preE = 0.0;
	float preF = 0.0;
	float preS = 0.0;
	std::smatch m;
	int lineIdx = 0;
	while (std::getline(file, line)) {
		GcodeItem gcode = GcodeItem();
		gcode.x = preX;
		gcode.y = preY;
		gcode.z = preZ;
		gcode.a = preA;
		gcode.b = preB;
		gcode.c = preC;
		gcode.e = preE;
		gcode.f = preF;
		gcode.s = preS;
		gcode.lineIdx = lineIdx;
		if (!std::regex_search(line, m, std::regex(";MACRO")))
		{
			if (std::regex_search(line, m, std::regex(";([0-9a-zA-Z-.\s_,:=]*)")))
			{
				gcode.comment = m[1].str();
				line = m.prefix();
			}
			if (std::regex_search(line, m, std::regex("^[G]([0-9]*)")))
			{
				gcode.gORm = true;
				int val = std::stoi(m[1].str());
				gcode.codeVal = val;
			}
			else if (std::regex_search(line, m, std::regex("^[M]([0-9]*)")))
			{
				gcode.gORm = false;
				int val = std::stoi(m[1].str());
				gcode.codeVal = val;
			}
			if (std::regex_search(line, m, std::regex("[X](-?[0-9]*.?[0-9]*)")))
			{
				float val = std::stof(m[1].str());
				gcode.x = val;
			}
			if (std::regex_search(line, m, std::regex("[Y](-?[0-9]*.?[0-9]*)")))
			{
				float val = std::stof(m[1].str());
				gcode.y = val;
			}
			if (std::regex_search(line, m, std::regex("[Z](-?[0-9]*.?[0-9]*)")))
			{
				float val = std::stof(m[1].str());
				gcode.z = val;
			}
			if (std::regex_search(line, m, std::regex("[A](-?[0-9]*.?[0-9]*)")))
			{
				float val = std::stof(m[1].str());
				gcode.a = val / 180.0 * M_PI;
			}
			if (std::regex_search(line, m, std::regex("[B](-?[0-9]*.?[0-9]*)")))
			{
				float val = std::stof(m[1].str());
				gcode.b = val / 180.0 * M_PI;
			}
			if (std::regex_search(line, m, std::regex("[C](-?[0-9]*.?[0-9]*)")))
			{
				float val = std::stof(m[1].str());
				gcode.c = val / 180.0 * M_PI;
			}
			if (std::regex_search(line, m, std::regex("[E](-?[0-9]*.?[0-9]*)")))
			{
				float val = std::stof(m[1].str());
				gcode.e = val;
			}
			if (std::regex_search(line, m, std::regex("[F](-?[0-9]*.?[0-9]*)")))
			{
				float val = std::stof(m[1].str());
				gcode.f = val;
			}
			if (std::regex_search(line, m, std::regex("[S](-?[0-9]*.?[0-9]*)")))
			{
				float val = std::stof(m[1].str());
				gcode.s = val;
			}
			std::vector<std::string> token = split(gcode.comment, ',');
			if (token.size() > 12)
			{
				gcode.thick = std::stof(token[7]);
				gcode.width = std::stof(token[8]);
				gcode.blockIdx = std::stoi(token[9]);
				gcode.layerIdx = std::stoi(token[10]);
				gcode.loopIdx = std::stoi(token[11]);
				gcode.curveIdx = std::stoi(token[12]);
				bufGcode.push_back(gcode);
			}
			else
			{
				if (bufGcode.size() > 0)
				{
					gcode.thick = bufGcode[bufGcode.size() - 1].thick;
					gcode.width = bufGcode[bufGcode.size() - 1].width;
					gcode.blockIdx = bufGcode[bufGcode.size() - 1].blockIdx;
					gcode.layerIdx = bufGcode[bufGcode.size() - 1].layerIdx;
					gcode.loopIdx = bufGcode[bufGcode.size() - 1].loopIdx;
					gcode.curveIdx = bufGcode[bufGcode.size() - 1].curveIdx;
					bufGcode.push_back(gcode);
				}
			}
			preX = gcode.x;
			preY = gcode.y;
			preZ = gcode.z;
			preA = gcode.a;
			preB = gcode.b;
			preC = gcode.c;
			preE = gcode.e;
			preF = gcode.f;
			preS = gcode.s;
			lineIdx++;
		}
	}
	file.close();

	return bufGcode;
}

std::vector<PathItem> convertGcode2Path(Kinematics* kin, std::vector<GcodeItem> gcode)
{
	int cnt = 0;
	std::vector<PathItem> paths;
	PathItem bufPath = kin->ForwardKinematics4Item(gcode[0]);
	if (gcode[0].gORm)
	{
		bufPath.type = PathType::PrintStart;
		paths.push_back(bufPath);
	}
	bool printingFlag = false;
	for (int i = 1; i < gcode.size(); i++)
	{
		float dist = kin->CalcDistance(gcode[i - 1], gcode[i]);
		int div = ceil(dist / CONVERT_PITCH);
		if (div > 1)
		{
			for (int j = 0; j < div; j++)
			{
				GcodeItem bufGcode;
				bufGcode.x = (gcode[i].x - gcode[i - 1].x) * j / (div - 1) + gcode[i - 1].x;
				bufGcode.y = (gcode[i].y - gcode[i - 1].y) * j / (div - 1) + gcode[i - 1].y;
				bufGcode.z = (gcode[i].z - gcode[i - 1].z) * j / (div - 1) + gcode[i - 1].z;
				bufGcode.a = (gcode[i].a - gcode[i - 1].a) * j / (div - 1) + gcode[i - 1].a;
				bufGcode.b = (gcode[i].b - gcode[i - 1].b) * j / (div - 1) + gcode[i - 1].b;
				bufGcode.c = (gcode[i].c - gcode[i - 1].c) * j / (div - 1) + gcode[i - 1].c;
				bufGcode.width = (gcode[i].width - gcode[i - 1].width) * j / (div - 1) + gcode[i - 1].width;
				bufGcode.thick = (gcode[i].thick - gcode[i - 1].thick) * j / (div - 1) + gcode[i - 1].thick;
				bufPath = kin->ForwardKinematics4Item(bufGcode);
				bufPath.cmdVal[0] = gcode[i].x;
				bufPath.cmdVal[1] = gcode[i].y;
				bufPath.cmdVal[2] = gcode[i].z;
				bufPath.cmdVal[3] = gcode[i].a;
				bufPath.cmdVal[4] = gcode[i].b;
				bufPath.cmdVal[5] = gcode[i].c;
				bufPath.cmdVal[6] = gcode[i].e;
				bufPath.cmdVal[7] = gcode[i].f;
				bufPath.moveVal[0] = bufGcode.x;
				bufPath.moveVal[1] = bufGcode.y;
				bufPath.moveVal[2] = bufGcode.z;
				bufPath.moveVal[3] = bufGcode.a;
				bufPath.moveVal[4] = bufGcode.b;
				bufPath.moveVal[5] = bufGcode.c;
				if (gcode[i].e > gcode[i - 1].e)
				{
					if (printingFlag)
					{
						bufPath.type = PathType::PrintMiddle;
					}
					else
					{
						bufPath.type = PathType::PrintStart;
					}
					printingFlag = true;
				}
				else
				{
					if (printingFlag)
					{
						paths[cnt - 1].type = PathType::PrintEnd;
						bufPath.type = PathType::Saving;
					}
					else
					{
						bufPath.type = PathType::Saving;
					}
					printingFlag = false;
				}
				bufPath.lineIdx = gcode[i].lineIdx;
				bufPath.blockIdx = gcode[i].blockIdx;
				bufPath.layerIdx = gcode[i].layerIdx;
				paths.push_back(bufPath);
				cnt++;
			}
		}
		else
		{
			GcodeItem bufGcode;
			bufGcode.x = gcode[i].x;
			bufGcode.y = gcode[i].y;
			bufGcode.z = gcode[i].z;
			bufGcode.a = gcode[i].a;
			bufGcode.b = gcode[i].b;
			bufGcode.c = gcode[i].c;
			bufGcode.width = gcode[i].width;
			bufGcode.thick = gcode[i].thick;
			bufPath = kin->ForwardKinematics4Item(bufGcode);
			bufPath.cmdVal[0] = gcode[i].x;
			bufPath.cmdVal[1] = gcode[i].y;
			bufPath.cmdVal[2] = gcode[i].z;
			bufPath.cmdVal[3] = gcode[i].a;
			bufPath.cmdVal[4] = gcode[i].b;
			bufPath.cmdVal[5] = gcode[i].c;
			bufPath.cmdVal[6] = gcode[i].e;
			bufPath.cmdVal[7] = gcode[i].f;
			bufPath.moveVal[0] = bufGcode.x;
			bufPath.moveVal[1] = bufGcode.y;
			bufPath.moveVal[2] = bufGcode.z;
			bufPath.moveVal[3] = bufGcode.a;
			bufPath.moveVal[4] = bufGcode.b;
			bufPath.moveVal[5] = bufGcode.c;
			bufPath.type = PathType::PrintMiddle;
			bufPath.lineIdx = gcode[i].lineIdx;
			bufPath.blockIdx = gcode[i].blockIdx;
			bufPath.layerIdx = gcode[i].layerIdx;
			if (gcode[i].gORm)
			{
				paths.push_back(bufPath);
				cnt++;
			}
			else
			{
				bufPath.type = PathType::Saving;
				paths.push_back(bufPath);
				cnt++;
			}
		}
	}
	return paths;
}