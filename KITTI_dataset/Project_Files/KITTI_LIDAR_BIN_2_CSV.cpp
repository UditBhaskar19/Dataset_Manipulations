#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
//#include <opencv2/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/features2d.hpp>
#include <map>

//#include "dataStructures.h"
//#include "lidarData.hpp"


using namespace std;

// Load Lidar points from a given location and store them in a vector
// ====================================================================

struct LidarPoint { // single lidar point in space
	double x, y, z, r; // x,y,z in [m], r is point reflectivity
};

void loadLidarFromFile(vector<LidarPoint> &lidarPoints, string filename)
{
	// allocate 4 MB buffer (only ~130*4*4 KB are needed)
	unsigned long num = 1000000;
	float *data = (float*)malloc(num * sizeof(float));

	// pointers
	float *px = data + 0;
	float *py = data + 1;
	float *pz = data + 2;
	float *pr = data + 3;

	// load point cloud
	FILE *stream;
	stream = fopen(filename.c_str(), "rb");
	num = fread(data, sizeof(float), num, stream) / 4;

	for (int32_t i = 0; i < num; i++) {
		LidarPoint lpt;
		lpt.x = *px; lpt.y = *py; lpt.z = *pz; lpt.r = *pr;
		lidarPoints.push_back(lpt);
		px += 4; py += 4; pz += 4; pr += 4;
	}
	fclose(stream);
	free(data);
}
// =======================================================================

int main(int argc, char* argv[])
{
	// lidar csv file location
	// -----------------------
	string lidarCSVdataPath = "F:/DATASET/KITTI/data_tracking_velodyne/training/velodyne/0000/CSV";

	// KITTI Lidar data location
	// -------------------------
	string dataPath = "F:/DATASET/KITTI/data_tracking_velodyne/training/velodyne";

	// lidar
	// -----
	string lidarBasePath = dataPath;
	string lidarScene = "0000";     // left camera, color
	string lidarFileType = ".bin";
	int lidarStartIndex = 0;          // first file index to load (assumes Lidar and camera names have identical naming convention)
	int lidarEndIndex = 153;        // last file index to load , last image 77
	int lidarStepWidth = 1;
	int lidarFillWidth = 6;          // no. of digits which make up the file index (e.g. img-000001.png)
	int nLidarData = lidarEndIndex - lidarStartIndex;

	for (size_t lidarIndex = 0; lidarIndex <= nLidarData; lidarIndex += lidarStepWidth) {

		// assemble filenames for current index
		// ------------------------------------
		ostringstream lidarNumber;
		lidarNumber << setfill('0') << setw(lidarFillWidth) << lidarStartIndex + lidarIndex;
		string lidarFullFilename = lidarBasePath + "/" + lidarScene + "/" + lidarNumber.str() + lidarFileType;

		// load 3D Lidar points from file
		// ------------------------------
		std::vector<LidarPoint> lidarPoints;
		loadLidarFromFile(lidarPoints, lidarFullFilename);

		// csv file
		// --------
		ofstream writeToCSV_lidar;
		string fileName_csv = lidarCSVdataPath + "/" + lidarNumber.str() + ".csv";
		writeToCSV_lidar.open(fileName_csv);

		for (auto it = lidarPoints.begin(); it != lidarPoints.end(); ++it) {
			float px = (*it).x;
			float py = (*it).y;
			float pz = (*it).z;
			float pr = (*it).r;

			writeToCSV_lidar << px << "," << py << "," << pz << "," << pr << endl;
		}
		writeToCSV_lidar.close();
	}

}