#pragma once
#include "HeaderFiles.h"

typedef struct Cube {
	double h, w, l, x0, y0, z0;
} Cube;
typedef struct Rectange {
	double x1, y1, x2, y2;
} Rectange;

typedef struct LidarPoint {
	int id;
	double x, y, z, r;
}LidarPoint;

typedef struct LidarPointProjection {
	int id;
	cv::Point pt;
} LidarPointProjections;

typedef struct BoundingBoxCamera {
	int boxID;                     /* measurement ID */
	int trackID;   // object ID
	Rectange roi; // 2D region of interest in image coordinates 
	int objectClassID;  // object classification
	double confidence;  // classification confidence
} BoundingBoxCamera;

typedef struct LidarCameraFusedObjectParam {
	BoundingBoxCamera obj;
	std::vector<LidarPoint> LidarPoints;    // a vector of lidar points which project into 2D image ROI ( Bounding Box )
	std::vector<cv::KeyPoint> KeyPoints;    // keypoints enclosed by 2D ROI (Boinding Box)
	std::vector<cv::DMatch> KeyPntMatches;  // keypoint matches enclosed by the image ROI (Bounding Box)
	std::vector<double> MatchQuality;       // Match quality of the keypoints
	double AvgMatchQuality;                 // average match quality
	double FusedMeasurementQuality;         // fused measurement quality
} LidarCameraFusedObjectParam;

typedef struct DataFrame {
	cv::Mat CameraImage;                  // image captured at time t
	std::vector<cv::KeyPoint> KeyPoints;  // Keypoints within the current image ROI
	cv::Mat Descriptors; 
	std::vector<cv::DMatch> KeyPntMatches;  // keypoint matches between current and the previous image ROI
	std::vector<LidarPoint> LidarPoints;    // a vector of lidar points synchronized with the current image
	std::vector<LidarCameraFusedObjectParam> Objects; // a vector of fused objects
	std::map<int, int> BoundingBoxMatches;   // association matrix
} DataFrame;

typedef struct BoundingBoxKITTIAnnotation {
	int frameID;
	int measID;
	int classID;
	int turnication;
	int occlusion;
	double objObservationAngle;
	Rectange Box2D;
	Cube Box3D;
	double yawAngle;
} BoundingBoxKITTIAnnotation;

typedef struct EgoImuGpsParam {
	double lat, lon, alt;
	double roll, pitch, yaw;
	double vn, ve, vf, vl, vu;
	double ax, ay, az;
	double af, al, au;
	double wx, wy, wz;
	double wf, wl, wu;
	double posacc, velacc;
	int navstatus, numsatellites;
	int posmode, velmode, orimode;
} EgoImuGpsParam;

typedef struct CallibrationParamKITTI {
	cv::Mat P0, P1, P2, P3; // projection matrix (3X4)
	cv::Mat TrLidarToCam;   // lidar to camera transfoemation (Rotation then translation) (4X4)
	cv::Mat TrImuToLidar;   // IMU to Lidar transformation (4X4)
	cv::Mat Rectification;  // Rectification matrix to correct distortion (4X4)
} CallibrationParam;

typedef struct LidarParam {
	double hFOVmin, hFOVmax;   // (-180, 180)
	double vFOVmin, vFOVmax;   // (-24.9, 2.0) 
	double hRes, vRes;
	double minDist;
	double maxDist;
	double scale;
} LidarParam;

typedef struct CameraParam {
	double hFOVmin, hFOVmax;   // (-90, 90)
	double vFOVmin, vFOVmax;   // (-24.9, 2.0), 
	double minDist;
	double maxDist;
} CameraParam;



