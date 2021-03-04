#pragma once
#include "HeaderFiles.h"
#include "DataStructuresLidarCamera.h"
#define PI 3.14
#define _USE_MATH_DEFINES

void DetectionsInFrame(const std::string dataPathAnnotation, const std::string sceneID,
	                   const int imgStartIndex, const int imgEndIndex,
                       const int imgStepWidth, const int imgFillWidth,
	                   std::vector<std::vector<BoundingBoxKITTIAnnotation>> &CameraDetections);
void CameraObjectDetections(const std::string dataPathCamera, const std::string sceneID,
	                        size_t  imgIndex, const int imgStartIndex, const int imgFillWidth,
	                        std::vector<BoundingBoxKITTIAnnotation> &CameraDetections,
	                        std::vector<BoundingBoxCamera> &ObjectBox, cv::Mat &frame);
void CameraDetectionsCache(std::vector<BoundingBoxKITTIAnnotation> &CameraDetections,
	                       std::vector<BoundingBoxCamera> &ObjectBox);
void VisualizeAnnotations(cv::Mat &frame,
	                      std::vector<BoundingBoxCamera> &boundingBoxes);
void LoadLidarFromFile(const std::string dataPathLidar, const std::string sceneID, 
	                   size_t  frameIdx, const int frameStartIndex, const int frameFillWidth, 
	                   std::vector<LidarPoint> &LidarPoints);
void LoadCallibrationData(const std::string dataPathCallibration, const std::string sceneID,
	                      cv::Mat &P0, cv::Mat &P1, cv::Mat &P2, cv::Mat &P3,
	                      cv::Mat &R_rect, cv::Mat &Tr_velo_cam, cv::Mat &Tr_imu_velo);
void ProjectLidarOnImage(cv::Mat &frame, std::vector<LidarPoint> &LidarPoints,
	                     cv::Mat &ProjMat, cv::Mat TransMat, cv::Mat RectMat,
	                     LidarParam &lidarCalib, CameraParam &camCalib);
void CreateCachedLidarAndCameraFrames(std::vector<cv::Mat> &CamFrames_cached, 
	                                  std::vector<std::vector<LidarPoint>> &LidarFrames_cached,
	                                  const int imgStartIndex, const int imgEndIndex,
	                                  const int imgStepWidth, const int imgFillWidth,
	                                  const std::string sceneID,
	                                  const std::string dataPathCamera,
	                                  const std::string dataPathLidar);
void AssociatePtCld2ObjBoundingBox(std::vector<LidarPointProjection> &LidarProjections,
	                               std::vector<BoundingBoxCamera> &ObjectBox,
	                               std::vector<std::vector<BoundingBoxCamera>> &CamLidarAssociations,
	                               std::vector<std::vector<LidarPointProjection>> &CamToLidarAssociations);