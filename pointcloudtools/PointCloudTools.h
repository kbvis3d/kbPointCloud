#pragma once

#include <vector>
#include <string>
#include <algorithm>

using namespace std;
typedef unsigned  char byte;

namespace Kbvis
{
    struct Punkt3d
    {
        double x, y, z;

        Punkt3d()
        {
            x = y = z = 0.f;
        }
        Punkt3d(double _x, double _y, double _z)
        {
            x = _x;
            y = _y;
            z = _z;
        }
    };

    enum BoxTyp
    {
        Empty,
        Normal,
        Infinity
    };

    struct Box3d
    {
        Punkt3d _min;
        Punkt3d _max;
        bool _berechnet;
        BoxTyp _typ;

        Box3d()
        {
            _berechnet = false;
            _typ = Empty;
        }
        Box3d(Punkt3d minVal, Punkt3d maxVal)
        {
            AddPoint(minVal);
            AddPoint(maxVal);
        }
        void Box3d::AddBox(Box3d box)
        {
            if (IsEmpty())
            {
                _min = box._min;
                _max = box._max;
            }
            else
            {
                _min.x = min(box._min.x, _min.x);
                _min.y = min(box._min.y, _min.y);
                _min.z = min(box._min.z, _min.z);
                _min.x = min(box._max.x, _min.x);
                _min.y = min(box._max.y, _min.y);
                _min.z = min(box._max.z, _min.z);
                _max.x = max(box._min.x, _max.x);
                _max.y = max(box._min.y, _max.y);
                _max.z = max(box._min.z, _max.z);
                _max.x = max(box._max.x, _max.x);
                _max.y = max(box._max.y, _max.y);
                _max.z = max(box._max.z, _max.z);
            }
        }
        void Box3d::AddPoint(Punkt3d pt)
        {
            AddPoint(pt.x, pt.y, pt.z);
        }
        void Box3d::AddPoint(double x, double y, double z)
        {
            _berechnet = true;
            if (IsEmpty())
            {
                _min.x = x;
                _min.y = y;
                _min.z = z;
                _max.x = x;
                _max.y = y;
                _max.z = z;
            }
            else
            {
                _min.x = min(_min.x, x);
                _min.y = min(_min.y, y);
                _min.z = min(_min.z, z);
                _max.x = max(_max.x, x);
                _max.y = max(_max.y, y);
                _max.z = max(_max.z, z);
            }
            _typ = Normal;
        }
        bool IsCalculated()
        {
            return _berechnet;
        }
        bool IsEmpty()
        {
            return _typ == Empty;
        }
        bool IsInfinity()
        {
            return (_typ == Infinity);
        }
        bool IsNormal()
        {
            return (_typ == Normal);
        }
        double Width()
        {
            if (!IsEmpty() && !IsInfinity())
            {
                return (_max.x - _min.x);
            }
            return 0.0;
        }
        double Height()
        {
            if (!IsEmpty() && !IsInfinity())
            {
                return (_max.y - _min.y);
            }
            return 0.0;
        }
        double Thickness()
        {
            if (!IsEmpty() && !IsInfinity())
            {
                return (_max.z - _min.z);
            }
            return 0.0;
        }
        double Zmax()
        {
            return _max.z;
        }
        double Zmin()
        {
            return _min.z;
        }
        void Clear()
        {
            _min.x = 0;
            _min.y = 0;
            _min.z = 0;
            _max.x = 0;
            _max.y = 0;
            _max.z = 0;
            _typ = Empty;
            _berechnet = false;
        }
    };
}
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
        All = 4294967295
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
    _declspec(dllexport) Kbvis::Box3d* GetBoundingBox3d();
    _declspec(dllexport) double** GetBufferBoxesActual();
    _declspec(dllexport) double* GetOffsetPoint();
    _declspec(dllexport) unsigned long GetError();
    _declspec(dllexport) float** ReadPoints(int* pufferanzahl, int** puffergrößen, unsigned int** pufferCodes, PointAttributes::PointAttribute selectedAttributes, float minDist);
}


