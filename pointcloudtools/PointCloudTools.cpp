
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
#include <ppl.h>
#include <exception>

using byte = unsigned char;
using namespace concurrency;
using namespace pdal;
using namespace pdal::Dimension;

struct Point3d
{
	double x, y, z;

	Point3d()
	{
		x = y = z = 0.f;
	}
	Point3d(double _x, double _y, double _z)
	{
		x = _x;
		y = _y;
		z = _z;
	}
};

enum BoxType
{
	Empty,
	Normal,
	Infinity
};

struct Box3d
{
	Point3d min;
	Point3d max;
	bool mCalculated;
	BoxType mType;

	Box3d()
	{
		mCalculated = false;
		mType = Empty;
	}
	Box3d(Point3d _min, Point3d _max)
	{
		AddPoint(_min);
		AddPoint(_max);
	}
	void Box3d::AddBox(Box3d box)
	{
		if (IsEmpty())
		{
			min = box.min;
			max = box.max;
		}
		else
		{
			min.x = min(box.min.x, min.x);
			min.y = min(box.min.y, min.y);
			min.z = min(box.min.z, min.z);
			min.x = min(box.max.x, min.x);
			min.y = min(box.max.y, min.y);
			min.z = min(box.max.z, min.z);
			max.x = max(box.min.x, max.x);
			max.y = max(box.min.y, max.y);
			max.z = max(box.min.z, max.z);
			max.x = max(box.max.x, max.x);
			max.y = max(box.max.y, max.y);
			max.z = max(box.max.z, max.z);
		}
	}
	void Box3d::AddPoint(Point3d pt)
	{
		AddPoint(pt.x, pt.y, pt.z);
	}
	void Box3d::AddPoint(double x, double y, double z)
	{
		mCalculated = true;
		if (IsEmpty())
		{
			min.x = x;
			min.y = y;
			min.z = z;
			max.x = x;
			max.y = y;
			max.z = z;
		}
		else
		{
			min.x = min(min.x, x);
			min.y = min(min.y, y);
			min.z = min(min.z, z);
			max.x = max(max.x, x);
			max.y = max(max.y, y);
			max.z = max(max.z, z);
		}
		mType = Normal;
	}
	bool IsCalculated()
	{
		return mCalculated;
	}
	bool IsEmpty()
	{
		return mType == Empty;
	}
	bool IsInfinity()
	{
		return (mType == Infinity);
	}
	bool IsNormal()
	{
		return (mType == Normal);
	}
	double Width()
	{
		if (!IsEmpty() && !IsInfinity())
		{
			return (max.x - min.x);
		}
		return 0.0;
	}
	double Height()
	{
		if (!IsEmpty() && !IsInfinity())
		{
			return (max.y - min.y);
		}
		return 0.0;
	}
	double Thickness()
	{
		if (!IsEmpty() && !IsInfinity())
		{
			return (max.z - min.z);
		}
		return 0.0;
	}
	double Zmax()
	{
		return max.z;
	}
	double Zmin()
	{
		return min.z;
	}
};

static float** spPunkte;
static Point3d spOffsetpunkt;
static Box3d spBegrenzungsbox;
static byte** spRgb;
static byte** spIntensitäten;
static byte** spTime;
static byte** spWave;
static byte** spInfrared;
static byte** spClassification;
static int snPunktwolkengröße;
static int snPufferanzahl;
static int *snPuffergrößen;
static bool sUseMinimumPointDistance;
static bool sStoreCodesOnly;
static bool sHasRGB, sHasIntensity, sHasClassification, sHasWave, sHasTime, sHasInfrared;
static bool sUseRGB, sUseIntensity, sUseClassification, sUseWave, sUseTime, sUseInfrared;
static float sMinimumPointDistance;
static string* sFilenames;
static int sFilecount;
static const int snBufferGranularity = 8;

struct PointColor{
	int r, g, b;

	PointColor()
	{
	}
	PointColor(int _r, int _g, int _b)
	{
		r = _r;
		g = _g;
		b = _b;
	}
};

struct PointList {
	float *pointList;
	int *colorList;
	int *intensityList;
	int *classificationList;
	int *gpsTimeList;
	int *waveList;
	int *infraredList;
	int count, capacity;

	PointList(int n, bool rgb, bool intensity)
	{
		count = 0;
		capacity = n;
		pointList = new float[n * 3];
		colorList = rgb ? new int[n * 3] : nullptr;
		intensityList = intensity ? new int[n] : nullptr;
	}
	~PointList()
	{
		//delete pointList;
		delete colorList;
		delete intensityList;
	}
};


extern "C"
{
	_declspec(dllexport) int OpenLasFiles(char** filenames, int filecount)
	{
		try
		{
			sFilenames = new string[filecount];
			sFilecount = filecount;
			snPunktwolkengröße = 0;
			sHasRGB = sHasIntensity = sHasClassification = sHasWave = sHasTime = sHasInfrared = true;
			spBegrenzungsbox = Box3d();

			for (int f = 0; f < filecount; ++f) {
				sFilenames[f] = string(filenames[f]);
				//pdal::Option las_opt("filename", string(filenames[f]));
				//pdal::Options las_opts(las_opt);
				pdal::Options las_opts;
				las_opts.add("filename", string(filenames[f]));
				//las_opts.add(las_opt);
				LasReader sLasReader;
				sLasReader.setOptions(las_opts);
				PointTable sTable;
				sLasReader.prepare(sTable);
				snPunktwolkengröße += sLasReader.getNumPoints();
				LasHeader sLasHeader = sLasReader.header();
				sHasInfrared = sHasInfrared && sLasHeader.hasInfrared();
				sHasRGB = sHasRGB && sLasHeader.hasColor();
				sHasTime = sHasTime&& sLasHeader.hasTime();
				sHasWave = sHasWave&&sLasHeader.hasWave();
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
		delete[] snPuffergrößen;
		for (int i = 0;i < snPufferanzahl;i++)
		{
			delete[] spPunkte[i];
			if (spRgb != nullptr) delete[]spRgb[i];
			if (spIntensitäten != nullptr) delete[]spIntensitäten[i];
		}
		delete[] spPunkte;
		if (spRgb != nullptr) delete[]spRgb;
		if (spIntensitäten != nullptr) delete[]spIntensitäten;
	}
	_declspec(dllexport) bool HasInfrared()
	{
		return sHasInfrared;
	}
	_declspec(dllexport) void UseInfrared(bool bUse)
	{
		sUseInfrared = bUse;
	}
	_declspec(dllexport) bool HasIntensity()
	{
		return sHasIntensity;
	}
	_declspec(dllexport) void UseIntensity(bool bUse)
	{
		sUseIntensity = bUse;
	}
	_declspec(dllexport) bool HasRGB()
	{
		return sHasRGB;
	}
	_declspec(dllexport) void UseRGB(bool bUse)
	{
		sUseRGB = bUse;
	}
	_declspec(dllexport) bool HasWave()
	{
		return sHasWave;
	}
	_declspec(dllexport) void UseWave(bool bUse)
	{
		sUseWave = bUse;
	}
	_declspec(dllexport) bool HasTime()
	{
		return sHasTime;
	}
	_declspec(dllexport) void UseTime(bool bUse)
	{
		sUseTime = bUse;
	}
	_declspec(dllexport) bool HasClassification()
	{
		return sHasClassification;
	}
	_declspec(dllexport) void UseClassification(bool bUse)
	{
		sUseClassification = bUse;
	}
	_declspec(dllexport) byte** GetPointRGBs()
	{
		return spRgb;
	}
	_declspec(dllexport) byte** GetPointIntensities()
	{
		return spIntensitäten;
	}
	_declspec(dllexport) int GetPointCloudSize()
	{
		return snPunktwolkengröße;
	}
	_declspec(dllexport) double* GetBoundingBox()
	{
		double *pBox = new double[6] {
			spBegrenzungsbox.min.x,
				spBegrenzungsbox.min.y,
				spBegrenzungsbox.min.z,
				spBegrenzungsbox.max.x,
				spBegrenzungsbox.max.y,
				spBegrenzungsbox.max.z
		};
		return pBox;
	}
	_declspec(dllexport) double* GetOffsetPoint()
	{
		double *pOP = new double[3] {
			spOffsetpunkt.x,
				spOffsetpunkt.x,
				spOffsetpunkt.x
		};
		return pOP;
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
		char *getPoint(PointId idx)
		{
			return NULL;
		}
		void setField(const Dimension::Detail *d, PointId idx, const void *value)
		{
			if (d->id() == Dimension::Id::X)
				m_x = *(const double *)value;
			else if (d->id() == Dimension::Id::Y)
				m_y = *(const double *)value;
			else if (d->id() == Dimension::Id::Z)
				m_z = *(const double *)value;
		}
		void getField(const Dimension::Detail *d, PointId idx, void *value)
		{
			if (d->id() == Dimension::Id::X)
				*(double *)value = m_x;
			else if (d->id() == Dimension::Id::Y)
				*(double *)value = m_y;
			else if (d->id() == Dimension::Id::Z)
				*(double *)value = m_z;
		}
	};

	_declspec(dllexport) float** ReadPoints(int *pufferanzahl, int **puffergrößen, unsigned int **pufferCodes, bool useMinimumPointDistance = false, bool storeCodesOnly = false, float minDist = 0.f)
	{
		//sUseTime = true;
		try
		{
			sUseMinimumPointDistance = useMinimumPointDistance;
			sStoreCodesOnly = storeCodesOnly;
			sMinimumPointDistance = minDist;

			float punktwolkeUmfang = max(max(spBegrenzungsbox.Width(), spBegrenzungsbox.Height()), spBegrenzungsbox.Thickness());
			const float skalierungsFaktor = float(snBufferGranularity) / punktwolkeUmfang;
			int rgbMax = 0, intensityMax = 0, gpsTimeMin = INT32_MAX, gpsTimeMax = 0, classificationMax = 0;

			spPunkte = nullptr;
			spRgb = nullptr;
			spIntensitäten = nullptr;
			int **pnIntensitäten = nullptr, **pnRGBs = nullptr, *pnIntMax = nullptr, *pnRgbMax = nullptr;
			map<unsigned int, PointList*> pPointLists;
			int bufferLimit = 10000000;
			int initialAlloc = 1000000;

			int codeRemap[snBufferGranularity*snBufferGranularity*snBufferGranularity];
			for (int c = 0; c < snBufferGranularity*snBufferGranularity*snBufferGranularity; ++c) {
				codeRemap[c] = -1;
			}

			auto cb = [&](PointRef& point) {

				double x = point.getFieldAs<double>(Id::X) - spOffsetpunkt.x;// -fileOffset.x;
				double y = point.getFieldAs<double>(Id::Y) - spOffsetpunkt.y;// -fileOffset.y;
				double z = point.getFieldAs<double>(Id::Z) - spOffsetpunkt.z;// -fileOffset.z;

				unsigned int ux = (unsigned int)((x - spBegrenzungsbox.min.x) * skalierungsFaktor);
				unsigned int uy = (unsigned int)((y - spBegrenzungsbox.min.y) * skalierungsFaktor);
				unsigned int uz = (unsigned int)((z - spBegrenzungsbox.min.z) * skalierungsFaktor);
				ux = min(ux, snBufferGranularity - 1);
				uy = min(uy, snBufferGranularity - 1);
				uz = min(uz, snBufferGranularity - 1);
				unsigned int mcode = MortonCode9(ux, uy, uz);
				PointList *pointList;

				if (codeRemap[mcode] < 0) {
					codeRemap[mcode] = mcode;
					pointList = pPointLists[mcode] = new PointList(initialAlloc, sHasRGB, sHasIntensity);
				}
				else {
					unsigned int mappedCode = codeRemap[mcode];
					pointList = pPointLists[mappedCode];
					int capacity = pointList->capacity;
					if (pointList->count == capacity) {
						mappedCode += 0x1 << 16;
						codeRemap[mcode] = mappedCode;
						pointList = pPointLists[mappedCode] = new PointList(min(capacity * 2, bufferLimit), sHasRGB, sHasIntensity);
					}
					//mcode = mappedCode;
				}

				int index = pointList->count;
				int tripleIndex = 3 * index;
				pointList->pointList[tripleIndex] = x;
				pointList->pointList[tripleIndex + 1] = y;
				pointList->pointList[tripleIndex + 2] = z;

				if (sHasIntensity && sUseIntensity) {
					int intensity = point.getFieldAs<int>(Id::Intensity);
					intensityMax = max(intensityMax, intensity);
					pointList->intensityList[index] = intensity;
				}
				if (sHasRGB && sUseRGB) {
					int red = point.getFieldAs<int>(Id::Red);
					int green = point.getFieldAs<int>(Id::Green);
					int blue = point.getFieldAs<int>(Id::Blue);
					rgbMax = max(rgbMax, max(red, max(green, blue)));
					pointList->colorList[tripleIndex] = red;
					pointList->colorList[tripleIndex + 1] = green;
					pointList->colorList[tripleIndex + 2] = blue;
				}
				//if (sHasClassification && sUseClassification) {
				//	int classification = point.getFieldAs<int>(Id::Classification);
				//	classificationMax = std::max(classificationMax, classification);
				//	pointList->classificationList[index] = classification;
				//}
				//if (sHasTime && sUseTime) {
				//	int gpsTime = point.getFieldAs<int>(Id::GpsTime);
				//	gpsTimeMax = std::max(gpsTimeMax, gpsTime);
				//	gpsTimeMin = std::min(gpsTimeMin, gpsTime);
				//	pointList->gpsTimeList[index] = gpsTime;
				//}
				++(pointList->count);

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
				scf.setCallback(cb);
				scf.setInput(sLasReader);
				scf.prepare(t);
				scf.execute(t);

				LasHeader sLasHeader = sLasReader.header();

				Point3d fileOffset(sLasHeader.offsetX(), sLasHeader.offsetY(), sLasHeader.offsetZ());

				if (f == 0) {
					//spOffsetpunkt.x = sPointView->getFieldAs<double>(Id::X, 0);
					//spOffsetpunkt.y = sPointView->getFieldAs<double>(Id::Y, 0);
					//spOffsetpunkt.z = sPointView->getFieldAs<double>(Id::Z, 0);
					//spOffsetpunkt = spBegrenzungsbox.min;
				}
			}

			map<unsigned int, int> cellCounts;
			for (auto const& element : pPointLists) {
				if ((element.second)->count > snPunktwolkengröße / 10000) {
					cellCounts[element.first] = (element.second)->count;
				}
			}

			sHasIntensity = sHasIntensity && intensityMax > 0;
			sHasRGB = sHasRGB && rgbMax > 0;
			sHasClassification = sHasClassification && classificationMax > 0;
			sHasTime = sHasTime && (gpsTimeMax - gpsTimeMin > 0);

			*pufferanzahl = snPufferanzahl = cellCounts.size();
			*puffergrößen = new int[snPufferanzahl];
			*pufferCodes = new unsigned int[snPufferanzahl];
			spPunkte = new float*[snPufferanzahl];
			if (sHasIntensity && sUseIntensity) {
				spIntensitäten = new byte*[snPufferanzahl];
			}
			if (sHasRGB && sUseRGB) {
				spRgb = new byte*[snPufferanzahl];
			}
			if (sHasClassification && sUseClassification) {
				spClassification = new byte*[snPufferanzahl];
			}
			if (sHasTime && sUseTime) {
				spTime = new byte*[snPufferanzahl];
			}
			if (sHasInfrared && sUseInfrared) {
				spInfrared = new byte*[snPufferanzahl];
			}
			if (sHasWave && sUseWave) {
				spWave = new byte*[snPufferanzahl];
			}

			int puffer = 0;

			for (auto const& element : cellCounts) {
				unsigned int baseCode = element.first & 0xffff;
				(*pufferCodes)[puffer] = baseCode;
				int ptCount = element.second;
				(*puffergrößen)[puffer] = ptCount;
				spPunkte[puffer] = pPointLists[element.first]->pointList;
				byte *intensitätenPtr = nullptr;
				if (sHasIntensity) {
					intensitätenPtr = spIntensitäten[puffer] = new byte[element.second];
				}
				byte *rgbPtr = nullptr;
				if (sHasRGB) {
					rgbPtr = spRgb[puffer] = new byte[element.second * 3];
				}
				byte *gpsTimePtr = nullptr;
				if (sHasTime) {
					gpsTimePtr = spTime[puffer] = new byte[element.second];
				}
				byte *classificationPtr = nullptr;
				if (sHasClassification) {
					classificationPtr = spClassification[puffer] = new byte[element.second];
				}

				if (pPointLists.count(element.first)) {

					if (sHasIntensity) {
						int *intensityList = pPointLists[element.first]->intensityList;
						for (int p = 0; p < ptCount; ++p) {
							*intensitätenPtr++ = byte(255.0 * intensityList[p] / float(intensityMax));
						}
					}
					if (sHasRGB) {
						int *clrList = pPointLists[element.first]->colorList;
						for (int p = 0; p < ptCount; ++p) {
							*rgbPtr++ = byte(255.0 * clrList[p * 3] / float(rgbMax));
							*rgbPtr++ = byte(255.0 * clrList[p * 3 + 1] / float(rgbMax));
							*rgbPtr++ = byte(255.0 * clrList[p * 3 + 2] / float(rgbMax));
						}
					}
					if (sHasTime) {
						int *gpsTimeList = pPointLists[element.first]->gpsTimeList;
						float range = gpsTimeMax - gpsTimeMin;
						for (int p = 0; p < ptCount; ++p) {
							*gpsTimePtr++ = byte(255.0 * (gpsTimeList[p] - gpsTimeMin) / range);
						}
					}
					if (sHasClassification) {
						int *classificationList = pPointLists[element.first]->classificationList;
						for (int p = 0; p < ptCount; ++p) {
							*classificationPtr++ = classificationList[p];
						}
					}
				}

				++puffer;
			}

			pPointLists.clear();

			return spPunkte;
		}
		catch (...)
		{
		}
		return nullptr;
	}
}

