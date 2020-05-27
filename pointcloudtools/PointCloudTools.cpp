//#define ENABLE_ATTRIBUTES

#include <pdal/PointView.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/Dimension.hpp>
#include <pdal/Options.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/io/BufferReader.hpp>
#include <pdal/io/LasReader.hpp>
#include <pdal/io/LasHeader.hpp>
#include <pdal/Filter.hpp>
#include <pdal/filters/StreamCallbackFilter.hpp>

#include <string>
#include <vector>
#include <unordered_set>
#include <ppl.h>
#include <exception>
#include "PointCloudTools.h"

using namespace concurrency;
using namespace pdal;
using namespace pdal::Dimension;

static map< PointAttributes::PointAttribute, pdal::Dimension::Id> sAttributeToDimension = {
    {PointAttributes::PointAttribute::Intensity, Id::Intensity},
    {PointAttributes::PointAttribute::EdgeOfFlightLine, Id::EdgeOfFlightLine},
    {PointAttributes::PointAttribute::Classification, Id::Classification },
    {PointAttributes::PointAttribute::ScanAngleRank, Id::ScanAngleRank },
    {PointAttributes::PointAttribute::Curvature, Id::Curvature },
    {PointAttributes::PointAttribute::Density, Id::Density },
    {PointAttributes::PointAttribute::GpsTime, Id::GpsTime },
    {PointAttributes::PointAttribute::InternalTime, Id::InternalTime},
    {PointAttributes::PointAttribute::OffsetTime, Id::OffsetTime},
    {PointAttributes::PointAttribute::Infrared, Id::Infrared},
    {PointAttributes::PointAttribute::StartPulse, Id::StartPulse},
    {PointAttributes::PointAttribute::ReflectedPulse, Id::ReflectedPulse},
    {PointAttributes::PointAttribute::HeightAboveGround, Id::HeightAboveGround},
    {PointAttributes::PointAttribute::Pitch, Id::Pitch},
    {PointAttributes::PointAttribute::Roll, Id::Roll},
    {PointAttributes::PointAttribute::PulseWidth, Id::PulseWidth},
    {PointAttributes::PointAttribute::Deviation, Id::Deviation},
    {PointAttributes::PointAttribute::PassiveSignal, Id::PassiveSignal},
    {PointAttributes::PointAttribute::BackgroundRadiation, Id::BackgroundRadiation},
    {PointAttributes::PointAttribute::Azimuth, Id::Azimuth},
    {PointAttributes::PointAttribute::WanderAngle, Id::WanderAngle},
    {PointAttributes::PointAttribute::ScanDirectionFlag, Id::ScanDirectionFlag},
    {PointAttributes::PointAttribute::PointId, Id::PointId},
    {PointAttributes::PointAttribute::Mark, Id::Mark},
    {PointAttributes::PointAttribute::Alpha, Id::Alpha},
    {PointAttributes::PointAttribute::EchoRange, Id::EchoRange}
};

enum PointCloudError
{
    None = 0,
    InvalidExtents = 1
};
inline PointCloudError operator|(PointCloudError a, PointCloudError b)
{
    return static_cast<PointCloudError>(static_cast<int>(a) | static_cast<int>(b));
}
inline PointCloudError operator&(PointCloudError a, PointCloudError b)
{
    return static_cast<PointCloudError>(static_cast<int>(a) & static_cast<int>(b));
}
inline PointCloudError operator~(PointCloudError a)
{
    return static_cast<PointCloudError>(~static_cast<int>(a));
}

static float** spPunkte;
static Kbvis::Punkt3d spOffsetpunkt;
static Kbvis::Box3d spBegrenzungsbox;
static int snPunktwolkengröße;
static int snPufferanzahl;
static PointAttributes::PointAttribute sAttributeVerfügbar;
static byte** spRgb;
static map<PointAttributes::PointAttribute, unsigned short**> spAttributesShort;
static map<PointAttributes::PointAttribute, void**> sAttributeArrays;
static bool sUseMinimumPointDistance, sComputeExtents;
static float sMinimumPointDistance;
static string* sFilenames;
static int sFilecount;
static int snBufferGranularity = 8, sunPaletteSize = 65536;
static map<PointAttributes::PointAttribute, double> sAttributeMin, sAttributeMax;
static map<PointAttributes::PointAttribute, unsigned int*> sHistogramm;
static const int MaxBufferGranularity = 8;
static PointCloudError sError;
static bool sFirstPoint = true;
static double** spBufferBoxesActual;

struct PunktFarbe {
    int r, g, b;

    PunktFarbe()
    {
    }
    PunktFarbe(int _r, int _g, int _b)
    {
        r = _r;
        g = _g;
        b = _b;
    }
};

struct PunkteList {
    float* punkteList;
    map< PointAttributes::PointAttribute, void*>attributeList;
    int count, capacity;

    PunkteList(int n, unordered_set<PointAttributes::PointAttribute> attributeSet)
    {
        count = 0;
        capacity = n;
        punkteList = new float[n * 3];

        for (auto att : attributeSet) {
            switch (att)
            {
            case PointAttributes::PointAttribute::RGB:
                attributeList[att] = new unsigned short[n * 3];
                break;
            case PointAttributes::PointAttribute::Intensity:
            case PointAttributes::PointAttribute::Classification:
            case PointAttributes::PointAttribute::Infrared:
                attributeList[att] = new unsigned short[n];
                break;
            case PointAttributes::PointAttribute::InternalTime:
            case PointAttributes::PointAttribute::GpsTime:
            case PointAttributes::PointAttribute::OffsetTime:
                attributeList[att] = new double[n];
                break;
            default:
                attributeList[att] = nullptr;
                break;
            }
        }
    }
    ~PunkteList()
    {
    }
};



extern "C"
{
    _declspec(dllexport) bool HasAttribute(PointAttributes::PointAttribute attribute)
    {
        return (sAttributeVerfügbar & attribute) != 0;
    }

    _declspec(dllexport) unsigned int GetAvailableAttributes()
    {
        return sAttributeVerfügbar;
    }

    _declspec(dllexport) int OpenLasFiles(char** filenames, int filecount)
    {
        try
        {
            sFilenames = new string[filecount];
            sFilecount = filecount;
            snPunktwolkengröße = 0;
            sAttributeVerfügbar = PointAttributes::PointAttribute::All;
            sError = None;
            spBegrenzungsbox.Clear();

            for (int f = 0; f < filecount; ++f) {
                sFilenames[f] = string(filenames[f]);
                pdal::Option las_opt("filename", string(filenames[f]));
                pdal::Options las_opts;
                las_opts.add(las_opt);
                LasReader sLasReader;
                sLasReader.setOptions(las_opts);
                PointTable sTable;
                sLasReader.prepare(sTable);
                snPunktwolkengröße += sLasReader.getNumPoints();
                LasHeader sLasHeader = sLasReader.header();
                if (!sLasHeader.hasColor()) sAttributeVerfügbar = sAttributeVerfügbar & ~PointAttributes::PointAttribute::RGB;
                if (!sLasHeader.hasInfrared())sAttributeVerfügbar = sAttributeVerfügbar & ~PointAttributes::PointAttribute::Infrared;
                if (!sLasHeader.hasTime()) {
                    sAttributeVerfügbar = sAttributeVerfügbar & ~PointAttributes::PointAttribute::GpsTime;
                    sAttributeVerfügbar = sAttributeVerfügbar & ~PointAttributes::PointAttribute::InternalTime;
                    sAttributeVerfügbar = sAttributeVerfügbar & ~PointAttributes::PointAttribute::OffsetTime;
                }
                spBegrenzungsbox.AddPoint(sLasHeader.minX(), sLasHeader.minY(), sLasHeader.minZ());
                spBegrenzungsbox.AddPoint(sLasHeader.maxX(), sLasHeader.maxY(), sLasHeader.maxZ());
            }
            return snPunktwolkengröße;
        }
        catch (exception& e)
        {
            cerr << e.what() << endl;
        }
        return 0;
    }
    _declspec(dllexport) void CloseLasFiles()
    {
        for (int i = 0; i < snPufferanzahl; i++) {
            delete[] spPunkte[i];
        }
        for (auto pair : spAttributesShort) {
            for (int i = 0; i < snPufferanzahl; i++) {
                delete[]pair.second[i];
            }
        }
        if (spRgb != nullptr) {
            for (int i = 0; i < snPufferanzahl; i++)
            {
                delete[]spRgb[i];
            }
        }
    }
    _declspec(dllexport) double GetMinAttribute(PointAttributes::PointAttribute attribute)
    {
        return sAttributeMin[attribute];
    }
    _declspec(dllexport) double GetMaxAttribute(PointAttributes::PointAttribute attribute)
    {
        return sAttributeMax[attribute];
    }
    _declspec(dllexport) unsigned int* GetHistogram(PointAttributes::PointAttribute attribute)
    {
        return sHistogramm.count(attribute) ? sHistogramm[attribute] : nullptr;
    }
    _declspec(dllexport) unsigned short** GetAttributeValues(PointAttributes::PointAttribute attribute)
    {
        if (HasAttribute(attribute)) {
            return (unsigned short**)sAttributeArrays[attribute];
        }
        return nullptr;
    }
    _declspec(dllexport) byte** GetPointRGBs()
    {
        return (byte**)GetAttributeValues(PointAttributes::PointAttribute::RGB);
    }
    _declspec(dllexport) int GetPointCloudSize()
    {
        return snPunktwolkengröße;
    }
    _declspec(dllexport) double* GetBoundingBox()
    {
        double* pBox = new double[6]{
            spBegrenzungsbox._min.x,
                spBegrenzungsbox._min.y,
                spBegrenzungsbox._min.z,
                spBegrenzungsbox._max.x,
                spBegrenzungsbox._max.y,
                spBegrenzungsbox._max.z
        };
        return pBox;
    }
    _declspec(dllexport) Kbvis::Box3d* GetBoundingBox3d()
    {
        return &spBegrenzungsbox;
    }
    _declspec(dllexport) double** GetBufferBoxesActual()
    {
        return spBufferBoxesActual;
    }
    _declspec(dllexport) double* GetOffsetPoint()
    {
        double* pOP = new double[3]{
            spOffsetpunkt.x,
            spOffsetpunkt.y,
            spOffsetpunkt.z
        };
        return pOP;
    }
    _declspec(dllexport) unsigned long GetError()
    {
        return sError;
    }

    // codiere die (x,y,z) Koordinaten (10 bits in jeder Dimension) in einen 32-bit Morton code
    // siehe: https://devblogs.nvidia.com/thinking-parallel-part-iii-tree-construction-gpu/

    // magische Bitmasken
    static unsigned int encodingMask[6] = { 0x000003ff, 0, 0x30000ff, 0x0300f00f, 0x30c30c3, 0x9249249 };
    static unsigned int decodingMask[6] = { 0, 0x000003ff, 0x30000ff, 0x0300f00f, 0x30c30c3, 0x9249249 };
    // teilen die bits des 10-bit Koordinatenwerts
    static unsigned int interleaveBits(unsigned int coord)
    {
        unsigned int x = coord & encodingMask[0];
        x = (x | x << 16) & encodingMask[2];
        x = (x | x << 8) & encodingMask[3];
        x = (x | x << 4) & encodingMask[4];
        x = (x | x << 2) & encodingMask[5];
        return x;
    }
    // kodiere x,y,z in 32-bit Morton code mit magischen Bits
    static unsigned int MortonCode(unsigned int x, unsigned int y, unsigned int z)
    {
        return interleaveBits(x) | (interleaveBits(y) << 1) | (interleaveBits(z) << 2);
    }
    // erhalten interleavte Bits, d.h. jedes dritte Bit
    static unsigned int getInterleavedBits(unsigned int mortonCode)
    {
        unsigned int x = mortonCode & decodingMask[5];
        x = (x ^ (x >> 2)) & decodingMask[4];
        x = (x ^ (x >> 4)) & decodingMask[3];
        x = (x ^ (x >> 8)) & decodingMask[2];
        x = (x ^ (x >> 16)) & decodingMask[1];
        return x;
    }
    // teilen den 32-bit Morton code in einzelne 10-bit Koordinaten
    static void MortonDecode(unsigned int mortonCode, unsigned int& x, unsigned int& y, unsigned int& z)
    {
        x = getInterleavedBits(mortonCode);
        y = getInterleavedBits(mortonCode >> 1);
        z = getInterleavedBits(mortonCode >> 2);
    }

    // kodiere x,y,z in 6-bit Morton code
    static unsigned int MortonCode6(unsigned int x, unsigned int y, unsigned int z)
    {
        unsigned int code = 0;
        code |= z & 0x1;
        code |= (z << 2) & 0x8;
        code |= (y << 1) & 0x2;
        code |= (y << 3) & 0x10;
        code |= (x << 2) & 0x4;
        code |= (x << 4) & 0x20;
        return code;
    }
    // kodiere x,y,z in 9-bit Morton code
    inline static unsigned int MortonCode9(unsigned int x, unsigned int y, unsigned int z)
    {
        unsigned int code = 0;
        code |= z & 0x1;
        code |= (z << 2) & 0x8;
        code |= (z << 4) & 0x40;
        code |= (y << 1) & 0x2;
        code |= (y << 3) & 0x10;
        code |= (y << 5) & 0x80;
        code |= (x << 2) & 0x4;
        code |= (x << 4) & 0x20;
        code |= (x << 6) & 0x100;
        return code;
    }
    // kodiere x,y,z in 12-bit Morton code
    static unsigned int MortonCode12(unsigned int x, unsigned int y, unsigned int z)
    {
        unsigned int code = 0;
        code |= z & 0x1;
        code |= (z << 2) & 0x8;
        code |= (z << 4) & 0x40;
        code |= (z << 6) & 0x200;
        code |= (y << 1) & 0x2;
        code |= (y << 3) & 0x10;
        code |= (y << 5) & 0x80;
        code |= (y << 7) & 0x400;
        code |= (x << 2) & 0x4;
        code |= (x << 4) & 0x20;
        code |= (x << 6) & 0x100;
        code |= (x << 8) & 0x800;
        return code;
    }
    // kodiere x,y,z in 15-bit Morton code
    static unsigned int MortonCode15(unsigned int x, unsigned int y, unsigned int z)
    {
        unsigned int code = 0;
        code |= z & 0x1;
        code |= (z << 2) & 0x8;
        code |= (z << 4) & 0x40;
        code |= (z << 6) & 0x200;
        code |= (z << 8) & 0x1000;
        code |= (y << 1) & 0x2;
        code |= (y << 3) & 0x10;
        code |= (y << 5) & 0x80;
        code |= (y << 7) & 0x400;
        code |= (y << 9) & 0x2000;
        code |= (x << 2) & 0x4;
        code |= (x << 4) & 0x20;
        code |= (x << 6) & 0x100;
        code |= (x << 8) & 0x800;
        code |= (x << 10) & 0x4000;
        return code;
    }

    class UserTable : public PointTable
    {
    private:
        double m_x;
        double m_y;
        double m_z;

    public:
        PointId addPoint()
        {
            return 0;
        }
        char* getPoint(PointId idx)
        {
            return NULL;
        }
        void setField(const Dimension::Detail* d, PointId idx, const void* value)
        {
            if (d->id() == Dimension::Id::X)
                m_x = *(const double*)value;
            else if (d->id() == Dimension::Id::Y)
                m_y = *(const double*)value;
            else if (d->id() == Dimension::Id::Z)
                m_z = *(const double*)value;
        }
        void getField(const Dimension::Detail* d, PointId idx, void* value)
        {
            if (d->id() == Dimension::Id::X)
                *(double*)value = m_x;
            else if (d->id() == Dimension::Id::Y)
                *(double*)value = m_y;
            else if (d->id() == Dimension::Id::Z)
                *(double*)value = m_z;
        }
    };

    _declspec(dllexport) float** ReadPoints(int* pufferanzahl, int** puffergrößen, unsigned int** pufferCodes, PointAttributes::PointAttribute selectedAttributes, float minDist)
    {
        try
        {
            snBufferGranularity = MaxBufferGranularity;
            sUseMinimumPointDistance = minDist > 0;
            sMinimumPointDistance = minDist;
            sComputeExtents = false;
            float pointCloudExtent = max(max(spBegrenzungsbox.Width(), spBegrenzungsbox.Height()), spBegrenzungsbox.Thickness());
            Kbvis::Box3d computedExtents;
            if (pointCloudExtent <= 0) {
                sComputeExtents = true;
                sUseMinimumPointDistance = false;
                snBufferGranularity = 1;
                sError = sError | InvalidExtents;
            }
            const float scaleFactor = pointCloudExtent > 0 ? float(snBufferGranularity) / pointCloudExtent : 1;
            const float tenBitRange = 1024;
            const float minimumPointDistance = pointCloudExtent > sMinimumPointDistance * tenBitRange ? tenBitRange / pointCloudExtent : 1 / sMinimumPointDistance;
            spPunkte = nullptr;
            spRgb = nullptr;
            sFirstPoint = true;
            map<unsigned int, PunkteList*> pPunkteListen;
            int initialAlloc = 1 << 20;
            int bufferLimit = initialAlloc << 3;
            int codeRemap[MaxBufferGranularity * MaxBufferGranularity * MaxBufferGranularity];
            for (int c = 0; c < snBufferGranularity * snBufferGranularity * snBufferGranularity; ++c) {
                codeRemap[c] = -1;
            }
            unordered_set<unsigned int> codeSet;
            map<unsigned int, Kbvis::Box3d> bufferBoxesActual;

            unordered_set<PointAttributes::PointAttribute> selectedAttributesSet;
            for (int i = 0; i < PointAttributes::MAX_ATTRIBUTES; ++i) {
                PointAttributes::PointAttribute attr = (PointAttributes::PointAttribute)(1 << i);
                if ((selectedAttributes & attr) != 0) {
                    selectedAttributesSet.insert(attr);
                    sAttributeMin[attr] = DBL_MAX;
                    sAttributeMax[attr] = 0;
                }
            }

            auto pointCallback = [&](PointRef& point) {

                double x = point.getFieldAs<double>(Id::X);
                double y = point.getFieldAs<double>(Id::Y);
                double z = point.getFieldAs<double>(Id::Z);
                if (sFirstPoint) {
                    spOffsetpunkt = Kbvis::Punkt3d(x, y, z);
                    sFirstPoint = false;
                }
                if (sComputeExtents) {
                    computedExtents.AddPoint(x, y, z);
                }

                if (sUseMinimumPointDistance) {
                    unsigned int uxx = (unsigned int)((x - spBegrenzungsbox._min.x) * minimumPointDistance);
                    unsigned int uyy = (unsigned int)((y - spBegrenzungsbox._min.y) * minimumPointDistance);
                    unsigned int uzz = (unsigned int)((z - spBegrenzungsbox._min.z) * minimumPointDistance);
                    const int granularity = 1024;
                    uxx = min(uxx, granularity - 1);
                    uyy = min(uyy, granularity - 1);
                    uzz = min(uzz, granularity - 1);
                    auto insertResult = codeSet.insert(MortonCode(uxx, uyy, uzz));
                    if (!insertResult.second) {
                        return true;
                    }
                }

                unsigned int ux = (unsigned int)((x - spBegrenzungsbox._min.x) * scaleFactor);
                unsigned int uy = (unsigned int)((y - spBegrenzungsbox._min.y) * scaleFactor);
                unsigned int uz = (unsigned int)((z - spBegrenzungsbox._min.z) * scaleFactor);
                unsigned int uCellLimit = (unsigned int)(snBufferGranularity - 1);
                ux = min(ux, uCellLimit);
                uy = min(uy, uCellLimit);
                uz = min(uz, uCellLimit);
                unsigned int mcode = MortonCode9(ux, uy, uz);
                PunkteList* punkteList;
                Kbvis::Box3d* currentBox;

                if (codeRemap[mcode] < 0) {
                    codeRemap[mcode] = mcode;
                    punkteList = pPunkteListen[mcode] = new PunkteList(initialAlloc, selectedAttributesSet);
                    bufferBoxesActual[mcode] = Kbvis::Box3d();
                    currentBox = &bufferBoxesActual[mcode];
                }
                else {
                    unsigned int mappedCode = codeRemap[mcode];
                    punkteList = pPunkteListen[mappedCode];
                    int capacity = punkteList->capacity;
                    if (punkteList->count == capacity) {
                        mappedCode += 0x1 << 16;
                        codeRemap[mcode] = mappedCode;
                        punkteList = pPunkteListen[mappedCode] = new PunkteList(min(capacity * 2, bufferLimit), selectedAttributesSet);
                        bufferBoxesActual[mappedCode] = Kbvis::Box3d();
                    }
                    currentBox = &bufferBoxesActual[mappedCode];
                }

                x -= spOffsetpunkt.x;
                y -= spOffsetpunkt.y;
                z -= spOffsetpunkt.z;

                currentBox->AddPoint(x, y, z);

                double attributeVal;
                int index = punkteList->count;
                int tripleIndex = 3 * index;
                punkteList->punkteList[tripleIndex] = x;
                punkteList->punkteList[tripleIndex + 1] = y;
                punkteList->punkteList[tripleIndex + 2] = z;

                for (auto att : selectedAttributesSet) {
                    bool ignore = false;
                    switch (att)
                    {
                    case PointAttributes::PointAttribute::RGB:
                    {
                        unsigned short red = point.getFieldAs<unsigned short>(Id::Red);
                        unsigned short green = point.getFieldAs<unsigned short>(Id::Green);
                        unsigned short blue = point.getFieldAs<unsigned short>(Id::Blue);
                        attributeVal = max(red, max(green, blue));
                        unsigned short* ptr = (unsigned short*)punkteList->attributeList[att];
                        ptr[tripleIndex] = red;
                        ptr[tripleIndex + 1] = green;
                        ptr[tripleIndex + 2] = blue;
                    }
                    break;
#ifdef ENABLE_ATTRIBUTES
                    case PointAttributes::PointAttribute::Classification:
                    {
                        byte byteVal = point.getFieldAs<byte>(Id::Classification);
                        unsigned short* shortPtr = (unsigned short*)punkteList->attributeList[att];
                        shortPtr[index] = byteVal;
                        attributeVal = byteVal;
                    }
                    break;
                    case PointAttributes::PointAttribute::Intensity:
                    case PointAttributes::PointAttribute::Infrared:
                    {
                        unsigned short shortVal = point.getFieldAs<unsigned short>(sAttributeToDimension[att]);
                        unsigned short* shortPtr = (unsigned short*)punkteList->attributeList[att];
                        shortPtr[index] = shortVal;
                        attributeVal = shortVal;
                    }
                    break;
                    case PointAttributes::PointAttribute::InternalTime:
                    case PointAttributes::PointAttribute::GpsTime:
                    case PointAttributes::PointAttribute::OffsetTime:
                    {
                        double doubleVal = point.getFieldAs<double>(sAttributeToDimension[att]);
                        double* doublePtr = (double*)punkteList->attributeList[att];
                        doublePtr[index] = doubleVal;
                        attributeVal = doubleVal;
                    }
                    break;
#endif
                    default:
                        ignore = true;
                        break;
                    }

                    if (!ignore) {
                        sAttributeMin[att] = min(sAttributeMin[att], attributeVal);
                        sAttributeMax[att] = max(sAttributeMax[att], attributeVal);
                    }
                }

                ++(punkteList->count);

                return true;
            };

            for (int f = 0; f < sFilecount; ++f) {

                pdal::Option las_opt("filename", sFilenames[f]);
                pdal::Options las_opts;
                las_opts.add(las_opt);
                LasReader sLasReader;
                sLasReader.setOptions(las_opts);

                FixedPointTable t(1000);
                StreamCallbackFilter scf;
                scf.setCallback(pointCallback);
                scf.setInput(sLasReader);
                scf.prepare(t);
                scf.execute(t);

                LasHeader sLasHeader = sLasReader.header();

                Kbvis::Punkt3d fileOffset(sLasHeader.offsetX(), sLasHeader.offsetY(), sLasHeader.offsetZ());
                if (sComputeExtents) {
                    spBegrenzungsbox = computedExtents;
                }
            }

            map<unsigned int, int> cellCounts;
            for (auto const& element : pPunkteListen) {
                if ((element.second)->count > snPunktwolkengröße / 10'000) {
                    cellCounts[element.first] = (element.second)->count;
                }
            }

            *pufferanzahl = snPufferanzahl = cellCounts.size();
            *puffergrößen = new int[snPufferanzahl];
            *pufferCodes = new unsigned int[snPufferanzahl];
            spPunkte = new float* [snPufferanzahl];
            spBufferBoxesActual = new double* [snPufferanzahl];

            sAttributeVerfügbar = selectedAttributes;
            unordered_set< PointAttributes::PointAttribute> availableAttributeSet;

            for (auto att : selectedAttributesSet) {
                if (sAttributeMax[att] <= sAttributeMin[att]) {
                    sAttributeVerfügbar = sAttributeVerfügbar & ~att;
                    continue;
                }
                availableAttributeSet.insert(att);
                switch (att)
                {
                case PointAttributes::PointAttribute::RGB:
                    spRgb = new byte * [snPufferanzahl];
                    sAttributeArrays[PointAttributes::PointAttribute::RGB] = (void**)spRgb;
                    sAttributeVerfügbar = sAttributeVerfügbar | PointAttributes::PointAttribute::Red;
                    sAttributeVerfügbar = sAttributeVerfügbar | PointAttributes::PointAttribute::Green;
                    sAttributeVerfügbar = sAttributeVerfügbar | PointAttributes::PointAttribute::Blue;
                    sHistogramm[PointAttributes::PointAttribute::Red] = new unsigned int[sunPaletteSize];
                    memset(sHistogramm[PointAttributes::PointAttribute::Red], 0, sunPaletteSize * sizeof(unsigned int));
                    sHistogramm[PointAttributes::PointAttribute::Green] = new unsigned int[sunPaletteSize];
                    memset(sHistogramm[PointAttributes::PointAttribute::Green], 0, sunPaletteSize * sizeof(unsigned int));
                    sHistogramm[PointAttributes::PointAttribute::Blue] = new unsigned int[sunPaletteSize];
                    memset(sHistogramm[PointAttributes::PointAttribute::Blue], 0, sunPaletteSize * sizeof(unsigned int));
                    break;
#ifdef ENABLE_ATTRIBUTES
                case PointAttributes::PointAttribute::Intensity:
                case PointAttributes::PointAttribute::Classification:
                case PointAttributes::PointAttribute::Infrared:
                case PointAttributes::PointAttribute::InternalTime:
                case PointAttributes::PointAttribute::GpsTime:
                case PointAttributes::PointAttribute::OffsetTime:
                    spAttributesShort[att] = new unsigned short* [snPufferanzahl];
                    sAttributeArrays[att] = (void**)spAttributesShort[att];
                    sHistogramm[att] = new unsigned int[sunPaletteSize];
                    memset(sHistogramm[att], 0, sunPaletteSize * sizeof(unsigned int));
                    break;
#endif
                default:
                    break;
                }
            }

            int puffer = 0;

            for (auto const& element : cellCounts) {
                unsigned int baseCode = element.first & 0xffff;
                (*pufferCodes)[puffer] = baseCode;
                int ptCount = element.second;
                (*puffergrößen)[puffer] = ptCount;
                spPunkte[puffer] = pPunkteListen[element.first]->punkteList;
                Kbvis::Box3d pufferBoxActual = bufferBoxesActual[element.first];
                spBufferBoxesActual[puffer] = new double[6]{
                    pufferBoxActual._min.x,pufferBoxActual._min.y,pufferBoxActual._min.z,
                    pufferBoxActual.Width(), pufferBoxActual.Height(), pufferBoxActual.Thickness() };
                unsigned short* attributePtr = nullptr;
                byte* rgbPtr = nullptr;

                for (auto att : availableAttributeSet) {
                    switch (att)
                    {
                    case PointAttributes::PointAttribute::RGB:
                        rgbPtr = spRgb[puffer] = new byte[element.second * 3];
                        break;
#ifdef ENABLE_ATTRIBUTES
                    case PointAttributes::PointAttribute::Classification:
                    case PointAttributes::PointAttribute::GpsTime:
                    case PointAttributes::PointAttribute::Infrared:
                    case PointAttributes::PointAttribute::Intensity:
                    case PointAttributes::PointAttribute::InternalTime:
                    case PointAttributes::PointAttribute::OffsetTime:
                        attributePtr = spAttributesShort[att][puffer] = new unsigned short[element.second];
                        break;
#endif
                    default:
                        break;
                    }

                    if (pPunkteListen.count(element.first)) {
                        switch (att)
                        {
                        case PointAttributes::PointAttribute::RGB:
                        {
                            unsigned short* clrList = (unsigned short*)pPunkteListen[element.first]->attributeList[att];
                            for (int p = 0; p < ptCount; ++p) {
                                unsigned short redVal = clrList[p * 3];
                                byte redValByte = byte(redVal / 256.0);
                                ++sHistogramm[PointAttributes::PointAttribute::Red][redVal];
                                *rgbPtr++ = redValByte;
                                unsigned short grnVal = clrList[p * 3 + 1];
                                byte grnValByte = byte(grnVal / 256.0);
                                ++sHistogramm[PointAttributes::PointAttribute::Green][grnVal];
                                *rgbPtr++ = grnValByte;
                                unsigned short bluVal = clrList[p * 3 + 2];
                                byte bluValByte = byte(bluVal / 256.0);
                                ++sHistogramm[PointAttributes::PointAttribute::Blue][bluVal];
                                *rgbPtr++ = bluValByte;
                            }
                        }
                        break;
#ifdef ENABLE_ATTRIBUTES
                        case PointAttributes::PointAttribute::Intensity:
                        case PointAttributes::PointAttribute::Classification:
                        case PointAttributes::PointAttribute::Infrared:
                        {
                            unsigned short* attributeList = (unsigned short*)pPunkteListen[element.first]->attributeList[att];
                            for (int p = 0; p < ptCount; ++p) {
                                *attributePtr = attributeList[p] - sAttributeMin[att];
                                ++sHistogramm[att][*attributePtr++];
                            }
                        }
                        break;
                        case PointAttributes::PointAttribute::InternalTime:
                        case PointAttributes::PointAttribute::GpsTime:
                        case PointAttributes::PointAttribute::OffsetTime:
                        {
                            double* attributeList = (double*)pPunkteListen[element.first]->attributeList[att];
                            for (int p = 0; p < ptCount; ++p) {
                                *attributePtr = unsigned short(UINT16_MAX * ((attributeList[p] - sAttributeMin[att]) / (sAttributeMax[att] - sAttributeMin[att])));
                                ++sHistogramm[att][*attributePtr++];
                            }
                        }
                        break;
#endif
                        default:
                            break;
                        }
                    }
                }
                ++puffer;
            }

            pPunkteListen.clear();

            return spPunkte;
        }
        catch (...)
        {
        }
        return nullptr;
    }
}
