#include "HeaderFiles.h"
#include "HelperFunctions.h"


// Global variables 1) data paths
const std::string dataPathCamera = "F:/KITTI/data_tracking_image_2/training/image_02/";
const std::string dataPathLidar = "F:/KITTI/data_tracking_velodyne/training/velodyne/";
const std::string dataPathAnnotations = "F:/KITTI/data_tracking_label_2/CSV/";
const std::string dataPathCallibration = "F:/KITTI/data_tracking_calib/training/calib/";
const std::string dataPathIMU = "F:/KITTI/data_tracking_oxts/training/oxts/";
// Global variables 2) scene parameters
const std::string scene = "0002";
const std::string imgFileType = ".png";
const int nImages = 153;
const int imgStartIndex = 0;   // first file index to load (assumes Lidar and camera names have identical naming convention)
const int imgEndIndex = 143;   // last file index to load
const int imgStepWidth = 1;
const int imgFillWidth = 6;    // no. of digits which make up the file index (e.g. img-0001.png)
double sensorFrameRate = 10.0 / imgStepWidth;
double dT = (1 / sensorFrameRate)*1000.0;
const int dataBufferSize = 10;
// Global variables 3) Callibration Parameters
cv::Mat P0(3, 4, cv::DataType<double>::type); // 3x4 projection matrix after rectification
cv::Mat P1(3, 4, cv::DataType<double>::type); // 3x4 projection matrix after rectification
cv::Mat P2(3, 4, cv::DataType<double>::type); // 3x4 projection matrix after rectification
cv::Mat P3(3, 4, cv::DataType<double>::type); // 3x4 projection matrix after rectification
cv::Mat R_rect(4, 4, cv::DataType<double>::type); // 3x3 rectifying rotation to make image planes co-planar
cv::Mat Tr_velo_cam(4, 4, cv::DataType<double>::type); // rotation matrix and translation vector
cv::Mat Tr_imu_velo(4, 4, cv::DataType<double>::type); // rotation matrix and translation vector

int main(int argc, char *argv[]) {
	std::vector<std::vector<BoundingBoxKITTIAnnotation>> CamDetGt;
	std::vector<BoundingBoxCamera> ObjDetections;
	cv::Mat frame;
	std::vector<LidarPoint> LidarPoints;
	double time_elapsed_ms, delay;
	LidarParam lidarCalib;
	CameraParam camCalib;

	std::vector<cv::Mat> CamFrames_cached;
	std::vector<std::vector<LidarPoint>> LidarFrames_cached;
	CreateCachedLidarAndCameraFrames(CamFrames_cached, LidarFrames_cached, imgStartIndex, imgEndIndex, imgStepWidth, imgFillWidth, scene, dataPathCamera, dataPathLidar);
	std::cout << "Sensor Data Loaded in the memory ! .." << std::endl;
	LoadCallibrationData(dataPathCallibration, scene, P0, P1, P2, P3, R_rect, Tr_velo_cam, Tr_imu_velo);
	std::cout << "Sensor Callibration Data Loaded in the memory ! .." << std::endl;
	DetectionsInFrame(dataPathAnnotations, scene, imgStartIndex, imgEndIndex, imgStepWidth, imgFillWidth, CamDetGt);
	std::cout << "Ground Truth data loaded ! .. PRESS ANY KEY TO CONTINUE" << std::endl;

	for (size_t idx = 0; idx <= imgEndIndex - imgStartIndex; idx += imgStepWidth) {
		std::clock_t startTime = std::clock(); // Timer Start

		CameraDetectionsCache(CamDetGt[idx], ObjDetections);
		VisualizeAnnotations(CamFrames_cached[idx], ObjDetections); // visualize the detections
		ProjectLidarOnImage(CamFrames_cached[idx], LidarFrames_cached[idx], P2, Tr_velo_cam, R_rect, lidarCalib, camCalib);
		ObjDetections.clear();
		LidarPoints.clear();

		std::clock_t endTime = std::clock();  // Timer End
		time_elapsed_ms = 1000.0 * (endTime - startTime) / CLOCKS_PER_SEC;
		delay = dT - time_elapsed_ms;
		delay = ((delay >= 1) ? delay : 1);
		std::cout << "CPU time used: " << time_elapsed_ms << " ms  , DELAY: " << delay << " ms " << std::endl;
		cv::waitKey(delay);
	}
	return 1;
}