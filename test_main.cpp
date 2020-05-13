#include "../pointcloudtools/PointCloudTools.h"
#include<iostream>

using namespace std;
typedef unsigned  char byte;

void main(int argc, char* argv[])
{
	//int pointcount = OpenLasFiles(new char*[1] {"test.las"}, 1, true, true);
	int pointcount = OpenLasFiles(new char*[1]{ "flower.las" }, 1, true, true);
	//int pointcount = OpenLasFiles(new char*[1]{ "statue.las" }, 1, true, true);
	//int pointcount = OpenLasFiles(new char*[1]{ "towerComplete.las" }, 1, true, true);
	//int pointcount = OpenLasFiles(new char*[1]{ "flight_scan_minegrid.laz" }, 1, true, true);

	UseRGB(HasRGB());
	UseIntensity(HasRGB());

	int bufferCount;
	int *bufferSizes;
	unsigned int *bufferCodes;
	float **ptSets = ReadPoints(&bufferCount, &bufferSizes, &bufferCodes);
	byte **prgbarrays = GetPointRGBs();
	byte **pintarrays = GetPointIntensities();
	int pcs = GetPointCloudSize();

	bool hasRGB = HasRGB();
	bool hasClassification = HasClassification();
	bool hasTime = HasTime();
	bool hasIR = HasInfrared();
	bool hasWave = HasWave();
	bool hasIntensity = HasIntensity();
	if (hasRGB) cerr << "RGB\n";
	if (hasIntensity) cerr << "Intensity\n";
	if (hasClassification) cerr << "Classification\n";
	if (hasTime) cerr << "Time\n";
	if (hasIR) cerr << "Infrared\n";
	if (hasWave) cerr << "Wave\n";

	cerr << "\n***************** " << bufferCount << " buffers ***************************\n";

	for (int i = 0; i < bufferCount; i++)
	{
		float *pts = ptSets[i];
		byte *prgb = prgbarrays != nullptr ? prgbarrays[i] : nullptr;
		byte *pints = pintarrays != nullptr ? pintarrays[i] : nullptr;

		cerr << "******************* " << i << " buffer size = " << bufferSizes[i] << endl;

		for (int j = 0; j < 2; j++){
			cerr << *pts++ << " ";
			cerr << *pts++ << " ";
			cerr << *pts++ << " ";
			if (prgb != nullptr) {
				cerr << int(*prgb++) << " ";
				cerr << int(*prgb++) << " ";
				cerr << int(*prgb++) << " ";
			}
			if (pints != nullptr) {
				cerr << int(*pints++);
			}
			cerr << endl;
		}

		double *pbox = GetBoundingBox();
		cerr << "\nBounding Box = " << pbox[0] << " " << pbox[1] << " " << pbox[2] << " " << pbox[3] << " " << pbox[4] << " " << pbox[5] << endl;
		double *poff = GetOffsetPoint();
		cerr << "\nOffsetPoint = " << poff[0] << " " << poff[1] << " " << poff[2] << endl;
	}
	CloseLasFiles();
}
