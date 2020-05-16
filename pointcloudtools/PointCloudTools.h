#pragma once

#include <vector>
#include <string>

using namespace std;
typedef unsigned  char byte;

namespace PointAttributes
{
    enum PointAttribute : unsigned long
    {
        None = 0,
        RGB = 1,
        Intensity = 2,
        EdgeOfFlightLine = 4,
        Classification = 8,
        ScanAngleRank = 16,
        Curvature = 32,
        Density = 64,
        Red = 128,
        GpsTime = 256,
        InternalTime = 512,
        OffsetTime = 1024,
        Infrared = 2048,
        StartPulse = 4096,
        ReflectedPulse = 8192,
        HeightAboveGround = 16384,
        Pitch = 32768,
        Roll = 65536,
        PulseWidth = 131072,
        Deviation = 262144,
        PassiveSignal = 524288,
        BackgroundRadiation = 1048576,
        Green = 2097152,
        Blue = 4194304,
        Azimuth = 8388608,
        WanderAngle = 16777216,
        ScanDirectionFlag = 33554432,
        ColorIndex = 67108864,
        PointId = 134217728,
        Mark = 268435456,
        Alpha = 536870912,
        EchoRange = 1073741824,
        All = 0xffffffffffffffff
    };
    inline PointAttribute operator|(PointAttribute a, PointAttribute b)
    {
        return static_cast<PointAttribute>(static_cast<int>(a) | static_cast<int>(b));
    }
    inline PointAttribute operator&(PointAttribute a, PointAttribute b)
    {
        return static_cast<PointAttribute>(static_cast<int>(a) & static_cast<int>(b));
    }
    inline PointAttribute operator~(PointAttribute a)
    {
        return static_cast<PointAttribute>(~static_cast<int>(a));
    }
    const int MAX_ATTRIBUTES = 31;
}

extern "C"
{
    _declspec(dllexport) bool HasAttribute(PointAttributes::PointAttribute attribute);
    _declspec(dllexport) unsigned int GetAvailableAttributes();
    _declspec(dllexport) int OpenLasFiles(char** filenames, int filecount);
    _declspec(dllexport) void CloseLasFiles();
    _declspec(dllexport) double GetMinAttribute(PointAttributes::PointAttribute attribute);
    _declspec(dllexport) double GetMaxAttribute(PointAttributes::PointAttribute attribute);
    _declspec(dllexport) unsigned int* GetHistogram(PointAttributes::PointAttribute attribute);
    _declspec(dllexport) unsigned short** GetAttributeValues(PointAttributes::PointAttribute attribute);
    _declspec(dllexport) byte** GetPointRGBs();
    _declspec(dllexport) int GetPointCloudSize();
    _declspec(dllexport) double* GetBoundingBox();
    _declspec(dllexport) double** GetBufferBoxesActual();
    _declspec(dllexport) double* GetOffsetPoint();
    _declspec(dllexport) unsigned long GetError();
    _declspec(dllexport) float** ReadPoints(int* pufferanzahl, int** puffergrößen, unsigned int** pufferCodes, PointAttributes::PointAttribute selectedAttributes, float minDist);
}


