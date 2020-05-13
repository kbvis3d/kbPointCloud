#pragma once

#include <vector>
#include <string>

using namespace std;
typedef unsigned  char byte;

extern "C"
{
	_declspec(dllimport) int OpenLasFiles(char** filenames, int filecount, bool getRgb, bool getIntensity);
	_declspec(dllimport) float** ReadPoints(int *pufferanzahl, int **puffergrößen, unsigned int **pufferCodes, bool useMinimumPointDistance = false, bool storeCodesOnly = false, float minDist = 0.f);
	_declspec(dllimport) byte** GetPointRGBs();
	_declspec(dllimport) byte** GetPointIntensities();
	_declspec(dllimport) int GetPointCloudSize();
	_declspec(dllimport) double* GetBoundingBox();
	_declspec(dllimport) double* GetOffsetPoint();
	_declspec(dllimport) void CloseLasFiles();
	_declspec(dllexport) bool HasInfrared();
	_declspec(dllexport) void UseInfrared(bool bUse);
	_declspec(dllexport) bool HasRGB();
	_declspec(dllexport) void UseRGB(bool bUse);
	_declspec(dllexport) bool HasWave();
	_declspec(dllexport) void UseWave(bool bUse);
	_declspec(dllexport) bool HasTime();
	_declspec(dllexport) void UseTime(bool bUse);
	_declspec(dllexport) bool HasClassification();
	_declspec(dllexport) void UseClassification(bool bUse);
	_declspec(dllexport) bool HasIntensity();
	_declspec(dllexport) void UseIntensity(bool bUse);
}


