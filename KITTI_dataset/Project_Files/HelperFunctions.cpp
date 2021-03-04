#include "HelperFunctions.h"

void AssociatePtCld2ObjBoundingBox(std::vector<LidarPointProjection> &LidarProjections,
	std::vector<BoundingBoxCamera> &ObjectBox,
	std::vector<std::vector<BoundingBoxCamera>> &LidarToCamAssociations,
	std::vector<std::vector<LidarPointProjection>> &CamToLidarAssociations) {
	double alpha = 0.0;                                      /* Shrink percentage */
	double shrinkFactor = alpha / 100.0;                     /* Shrink factor */
	double shrinkFactorWidth, shrinkFactorHeight;            /* Amount by which to shrink the Bounding Box width and height */
	Rectange roi;                                            /* ROI around a camera detected object */
	std::vector<BoundingBoxCamera> AssociatedBox;            /* temporary variable*/
	for (std::vector<LidarPointProjection>::iterator ptIdx = LidarProjections.begin(); ptIdx != LidarProjections.end(); ptIdx++) {
		for (std::vector<BoundingBoxCamera>::iterator boxIdx = ObjectBox.begin(); boxIdx != ObjectBox.end(); boxIdx++) {
			shrinkFactorWidth = shrinkFactor * (boxIdx->roi.x2 - boxIdx->roi.x1);
			shrinkFactorHeight = shrinkFactor * (boxIdx->roi.y2 - boxIdx->roi.y1);
			roi.x1 = boxIdx->roi.x1 + shrinkFactorWidth / 2;
			roi.y1 = boxIdx->roi.y1 + shrinkFactorHeight / 2;
			roi.x2 = boxIdx->roi.x2 - shrinkFactorWidth / 2;
			roi.y2 = boxIdx->roi.y2 - shrinkFactorHeight / 2;
			if (ptIdx->pt.x > roi.x1 && ptIdx->pt.x < roi.x2 && ptIdx->pt.y > roi.y1 && ptIdx->pt.y < roi.y2) {
				AssociatedBox.push_back(*boxIdx);
			}
		}
		LidarToCamAssociations.push_back(AssociatedBox);
		AssociatedBox.clear();
	}
	for (std::vector<BoundingBoxCamera>::iterator boxIdx = ObjectBox.begin(); boxIdx != ObjectBox.end(); boxIdx++) {
		for (std::vector<LidarPointProjection>::iterator ptIdx = LidarProjections.begin(); ptIdx != LidarProjections.end(); ptIdx++) {
			shrinkFactorWidth = shrinkFactor * (boxIdx->roi.x2 - boxIdx->roi.x1);
			shrinkFactorHeight = shrinkFactor * (boxIdx->roi.y2 - boxIdx->roi.y1);
			roi.x1 = boxIdx->roi.x1 + shrinkFactorWidth / 2;
			roi.y1 = boxIdx->roi.y1 + shrinkFactorHeight / 2;
			roi.x2 = boxIdx->roi.x2 - shrinkFactorWidth / 2;
			roi.y2 = boxIdx->roi.y2 - shrinkFactorHeight / 2;
			if (ptIdx->pt.x > roi.x1 && ptIdx->pt.x < roi.x2 && ptIdx->pt.y > roi.y1 && ptIdx->pt.y < roi.y2) {
				AssociatedBox.push_back(*boxIdx);
			}
		}
		//CamToLidarAssociations.push_back(AssociatedBox);
		AssociatedBox.clear();
	}
}
// =======================================================================================================================================
void CreateCachedLidarAndCameraFrames(std::vector<cv::Mat> &CamFrames_cached,
	std::vector<std::vector<LidarPoint>> &LidarFrames_cached,
	const int imgStartIndex, const int imgEndIndex,
	const int imgStepWidth, const int imgFillWidth,
	const std::string sceneID,
	const std::string dataPathCamera,
	const std::string dataPathLidar) {
	// Create Camera Buffer
	cv::Mat frame;
	std::string fileTypeImg = ".png";
	for (size_t idx = imgStartIndex; idx <= imgEndIndex; idx += imgStepWidth) {
		std::ostringstream imgNumber;
		imgNumber << std::setfill('0') << std::setw(imgFillWidth) << imgStartIndex + idx;
		std::string FullFilename = dataPathCamera + sceneID + "/" + imgNumber.str() + fileTypeImg;
		//std::cout << FullFilename << std::endl;
		frame = cv::imread(FullFilename);
		CamFrames_cached.push_back(frame);
	}
	std::cout << "CAMERA Data Loaded in the memory ! .." << std::endl;
	// Create Lidar Buffer
	std::vector<LidarPoint> LidarPoints;
	for (size_t idx = imgStartIndex; idx <= imgEndIndex; idx += imgStepWidth) {
		LoadLidarFromFile(dataPathLidar, sceneID, idx, imgStartIndex, imgFillWidth, LidarPoints);
		LidarFrames_cached.push_back(LidarPoints);
		LidarPoints.clear();
	}
	std::cout << "LIDAR Data Loaded in the memory ! .." << std::endl;
}

void ProjectLidarOnImage(cv::Mat &frame, std::vector<LidarPoint> &LidarPoints,
	cv::Mat &ProjMat, cv::Mat TransMat, cv::Mat RectMat,
	LidarParam &lidarCalib, CameraParam &camCalib) {
	cv::Point pt;
	cv::Mat frameCpy = frame.clone();
	cv::Mat X(4, 1, cv::DataType<double>::type); //lidar input
	cv::Mat Y(3, 1, cv::DataType<double>::type); //lidar point after projection
	double hFOVmax = 45.0*PI / 180;  double hFOVmin = -45 * PI / 180;
	double vFOVmax = 2.0*PI / 180;   double vFOVmin = -24.9*PI / 180;
	double rangeMax = 80.0; double rangeMin = 0.0;
	double range, azimuth, elevation;
	int red, green, blue, val, maxVal;
	std::vector<cv::Point> ProjectedPts;
	for (auto it = LidarPoints.begin(); it != LidarPoints.end(); ++it) {
		// Select those Lidar points such that those are within the camera FOV
		range = sqrt(it->x * it->x + it->y * it->y + it->z * it->z);
		azimuth = atan2(it->x, it->y);
		elevation = atan2(range, it->z);
		/*
		if (azimuth > hFOVmin && azimuth < hFOVmax &&
			elevation > vFOVmin && elevation < vFOVmax &&
			range < rangeMax && range > rangeMin) */
		if (range < rangeMax && range > rangeMin && it->x > 0.0)
		{
			X.at<double>(0, 0) = it->x;
			X.at<double>(1, 0) = it->y;
			X.at<double>(2, 0) = it->z;
			X.at<double>(3, 0) = 1.0;
			// Y = ProjMat * RectMat * TransMat * X;
			Y = ProjMat * TransMat * X;
			pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0);
			pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0);
			ProjectedPts.push_back(pt);
			range = MIN(rangeMax, MAX(0, range));                    // clip the value between an uppder and lower
			// Projection Visualization
			int thickness = 2;
			if (range <= 5) {  // RED color
				red = ((range - 0) / (5 - 0)) * 255;
				green = 0;
				blue = 0;
				cv::circle(frameCpy, pt, thickness, cv::Scalar(blue, green, red), -1);
			}
			else if (range > 5 && range <= 10) {  // RED To ORANGE
				red = 255;
				green = ((range - 5) / (10 - 5)) * 140;
				blue = 0;
				cv::circle(frameCpy, pt, thickness, cv::Scalar(blue, green, red), -1);
			}
			else if (range > 10 && range <= 15) {  // ORANGE to YELLOW
				red = 255;
				green = 140 + ((range - 10) / (15 - 10)) * (255 - 140);
				blue = 0;
				cv::circle(frameCpy, pt, thickness, cv::Scalar(blue, green, red), -1);
			}
			else if (range > 15 && range <= 20) {  // YELLOW to LIGHT GREEN
				red = 255 - ((range - 15) / (20 - 15)) * (255 - 140);
				green = 255;
				blue = 0;
				cv::circle(frameCpy, pt, thickness, cv::Scalar(blue, green, red), -1);
			}
			else if (range > 20 && range <= 30) {
				red = 140 - ((range - 20) / (30 - 20)) * 140;
				green = 255;
				blue = 0;
				cv::circle(frameCpy, pt, thickness, cv::Scalar(blue, green, red), -1);
			}
			else if (range > 30 && range <= 40) {
				red = 0;
				green = 255;
				blue = ((range - 30) / (40 - 30)) * 255;
				cv::circle(frameCpy, pt, thickness, cv::Scalar(blue, green, red), -1);
			}
			else if (range > 40 && range <= 80) {
				red = 0;
				green = 255 - ((range - 40) / (80 - 40)) * 255;
				blue = 255;
				cv::circle(frameCpy, pt, thickness, cv::Scalar(blue, green, red), -1);
			}
		}
	}
	// Display the lidar projection
	std::string windowName = "Lidar Projection";
	cv::namedWindow(windowName, 1);
	cv::imshow(windowName, frameCpy);
}

void LoadLidarFromFile(const std::string dataPathLidar, const std::string sceneID,
	size_t  frameIdx, const int frameStartIndex, const int frameFillWidth,
	std::vector<LidarPoint> &LidarPoints) {
	std::string fileType = ".bin";
	std::ostringstream frameNumber;
	frameNumber << std::setfill('0') << std::setw(frameFillWidth) << frameStartIndex + frameIdx;
	std::string FullFilename = dataPathLidar + sceneID + "/" + frameNumber.str() + fileType;

	// allocate 4 MB buffer (only ~130*4*4 KB are needed)
	unsigned long num = 1000000;
	float *data = (float*)malloc(num * sizeof(float));
	// pointers
	float *px = data + 0;
	float *py = data + 1;
	float *pz = data + 2;
	float *pr = data + 3;
	// load point cloud from file
	FILE *stream;
	stream = fopen(FullFilename.c_str(), "rb");
	num = fread(data, sizeof(float), num, stream) / 4;
	// Save it to a structure; 
	int id;
	id = 1;
	for (int32_t i = 0; i < num; i++) {
		LidarPoint lpt;
		lpt.id = id;
		lpt.x = *px; lpt.y = *py; lpt.z = *pz; lpt.r = *pr;
		LidarPoints.push_back(lpt);
		px += 4; py += 4; pz += 4; pr += 4;
	}
	fclose(stream);
	free(data);
}

void VisualizeAnnotations(cv::Mat &frame, std::vector<BoundingBoxCamera> &boundingBoxes) {
	std::string windowName = "Object classification";
	cv::Mat frameCpy = frame.clone();
	for (auto it = boundingBoxes.begin(); it != boundingBoxes.end(); ++it) {
		int x1 = (*it).roi.x1;
		int y1 = (*it).roi.y1;
		int x2 = (*it).roi.x2;
		int y2 = (*it).roi.y2;
		cv::rectangle(frameCpy, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2);
		cv::namedWindow(windowName, 1);
		cv::imshow(windowName, frameCpy);
	}
}

void DetectionsInFrame(const std::string dataPathAnnotation, const std::string sceneID,
	const int imgStartIndex, const int imgEndIndex,
	const int imgStepWidth, const int imgFillWidth,
	std::vector<std::vector<BoundingBoxKITTIAnnotation>> &CameraDetections) {
	std::vector<BoundingBoxKITTIAnnotation> FrameDetection;
	BoundingBoxKITTIAnnotation ObjectDetection;
	std::string fileType = ".csv";
	int id;
	for (size_t idx = imgStartIndex; idx <= imgEndIndex; idx += imgStepWidth) {
		// assemble filenames for current index
		std::ostringstream imgNumber;
		imgNumber << std::setfill('0') << std::setw(imgFillWidth) << imgStartIndex + idx;
		std::string FullFilename = dataPathAnnotation + sceneID + "/" + imgNumber.str() + fileType;
		// Read the annotation data;
		std::ifstream Detection_data(FullFilename);
		if (!Detection_data.is_open()) {
			std::exit(EXIT_FAILURE);
		}
		std::string str;
		std::string str_temp;
		std::getline(Detection_data, str); // skip the first line
		id = 1;
		while (std::getline(Detection_data, str)) {
			std::stringstream ss(str);

			std::getline(ss, str_temp, ',');
			ObjectDetection.frameID = stoi(str_temp);

			std::getline(ss, str_temp, ',');
			//ObjectDetection.measID = stoi(str_temp);
			ObjectDetection.measID = id;
			id++;

			std::getline(ss, str_temp, ',');
			//ObjectDetection.classID = stoi(str_temp);
			ObjectDetection.classID = -1;

			std::getline(ss, str_temp, ',');
			ObjectDetection.turnication = stoi(str_temp);

			std::getline(ss, str_temp, ',');
			ObjectDetection.occlusion = stoi(str_temp);

			std::getline(ss, str_temp, ',');
			ObjectDetection.objObservationAngle = stod(str_temp);

			std::getline(ss, str_temp, ',');
			ObjectDetection.Box2D.x1 = stod(str_temp);

			std::getline(ss, str_temp, ',');
			ObjectDetection.Box2D.y1 = stod(str_temp);

			std::getline(ss, str_temp, ',');
			ObjectDetection.Box2D.x2 = stod(str_temp);

			std::getline(ss, str_temp, ',');
			ObjectDetection.Box2D.y2 = stod(str_temp);

			std::getline(ss, str_temp, ',');
			ObjectDetection.Box3D.h = stod(str_temp);

			std::getline(ss, str_temp, ',');
			ObjectDetection.Box3D.w = stod(str_temp);

			std::getline(ss, str_temp, ',');
			ObjectDetection.Box3D.l = stod(str_temp);

			std::getline(ss, str_temp, ',');
			ObjectDetection.Box3D.x0 = stod(str_temp);

			std::getline(ss, str_temp, ',');
			ObjectDetection.Box3D.y0 = stod(str_temp);

			std::getline(ss, str_temp, ',');
			ObjectDetection.Box3D.z0 = stod(str_temp);

			std::getline(ss, str_temp, ',');
			ObjectDetection.yawAngle = stod(str_temp);

			FrameDetection.push_back(ObjectDetection);
		}
		CameraDetections.push_back(FrameDetection);
		//std::cout << " Current Frame has : " << CameraDetections[idx].size() << " number of object " << std::endl;
		FrameDetection.clear();
	}
	//std::cout << " Total Number of frames is : " << CameraDetections.size() << std::endl;
}

void CameraObjectDetections(const std::string dataPathCamera, const std::string sceneID,
	size_t  imgIndex, const int imgStartIndex, const int imgFillWidth,
	std::vector<BoundingBoxKITTIAnnotation> &CameraDetections,
	std::vector<BoundingBoxCamera> &ObjectBox, cv::Mat &frame) {
	BoundingBoxCamera box;

	std::ostringstream imgNumber;
	std::string fileType = ".png";
	imgNumber << std::setfill('0') << std::setw(imgFillWidth) << imgStartIndex + imgIndex;
	std::string FullFilename = dataPathCamera + sceneID + "/" + imgNumber.str() + fileType;
	frame = cv::imread(FullFilename);

	for (auto it = CameraDetections.begin(); it != CameraDetections.end(); ++it) {
		box.objectClassID = (*it).classID;
		box.boxID = (*it).measID;
		box.roi.x1 = (*it).Box2D.x1;
		box.roi.y1 = (*it).Box2D.y1;
		box.roi.x2 = (*it).Box2D.x2;
		box.roi.y2 = (*it).Box2D.y2;
		box.confidence = 0.0;
		box.trackID = -1;
		ObjectBox.push_back(box);
	}
}

void CameraDetectionsCache(std::vector<BoundingBoxKITTIAnnotation> &CameraDetections,
	std::vector<BoundingBoxCamera> &ObjectBox) {
	BoundingBoxCamera box;
	for (auto it = CameraDetections.begin(); it != CameraDetections.end(); ++it) {
		box.objectClassID = (*it).classID;
		box.boxID = (*it).measID;
		box.roi.x1 = (*it).Box2D.x1;
		box.roi.y1 = (*it).Box2D.y1;
		box.roi.x2 = (*it).Box2D.x2;
		box.roi.y2 = (*it).Box2D.y2;
		box.confidence = 0.0;
		box.trackID = -1;
		ObjectBox.push_back(box);
	}
}

void LoadCallibrationData(const std::string dataPathCallibration, const std::string sceneID,
	cv::Mat &P0, cv::Mat &P1, cv::Mat &P2, cv::Mat &P3,
	cv::Mat &R_rect, cv::Mat &Tr_velo_cam, cv::Mat &Tr_imu_velo) {
	std::string fileType = ".txt";    // callibration file
	char delimiter = ' ';             // word seperator
	std::vector<std::string> words;   // used for parsing a line word by word
	std::vector<std::vector<std::string>> callib;
	std::string line, str_tmp;
	std::string FullFilename = dataPathCallibration + sceneID + fileType;   // callibration file
	std::ifstream CallibData(FullFilename);
	if (!CallibData.is_open()) {
		std::exit(EXIT_FAILURE);
	}
	while (std::getline(CallibData, line)) {
		std::stringstream ss(line);
		while (std::getline(ss, str_tmp, delimiter)) {
			if (str_tmp.size() != 0)
				words.push_back(str_tmp);
		}
		callib.push_back(words);
		words.clear();
	}
	std::cout << "Path : " << FullFilename << std::endl;
	std::cout << "============================================================================" << std::endl;
	std::cout << "Num of Param is : " << callib.size() << std::endl;
	std::cout << callib[0][0] << " Data size is : " << callib[0].size() - 1 << std::endl;
	std::cout << callib[1][0] << " Data size is : " << callib[1].size() - 1 << std::endl;
	std::cout << callib[2][0] << " Data size is : " << callib[2].size() - 1 << std::endl;
	std::cout << callib[3][0] << " Data size is : " << callib[3].size() - 1 << std::endl;
	std::cout << callib[4][0] << " Data size is : " << callib[4].size() - 1 << std::endl;
	std::cout << callib[5][0] << " Data size is : " << callib[5].size() - 1 << std::endl;
	std::cout << callib[6][0] << " Data size is : " << callib[6].size() - 1 << std::endl;
	std::cout << "=============================================================================" << std::endl;

	// Parameter set for P0 to P3 (3 x 4)
	int rowIdx, colIdx, idx;
	for (rowIdx = 0; rowIdx < 3; rowIdx++) {
		for (colIdx = 0; colIdx < 4; colIdx++) {
			idx = rowIdx * 4 + colIdx + 1;
			P0.at<double>(rowIdx, colIdx) = std::stod(callib[0][idx]);
			P1.at<double>(rowIdx, colIdx) = std::stod(callib[1][idx]);
			P2.at<double>(rowIdx, colIdx) = std::stod(callib[2][idx]);
			P3.at<double>(rowIdx, colIdx) = std::stod(callib[3][idx]);
		}
	}
	// Parameter set for R_rect (3 x 3) pad to set the dimension (4 x 4)
	for (rowIdx = 0; rowIdx < 3; rowIdx++) {
		for (colIdx = 0; colIdx < 3; colIdx++) {
			idx = rowIdx * 3 + colIdx + 1;
			R_rect.at<double>(rowIdx, colIdx) = std::stod(callib[4][idx]);
		}
		R_rect.at<double>(rowIdx, 3) = 0.0;
	}
	R_rect.at<double>(3, 0) = 0.0;
	R_rect.at<double>(3, 1) = 0.0;
	R_rect.at<double>(3, 2) = 0.0;
	R_rect.at<double>(3, 3) = 1.0;
	// Parameter set for Coordinate Transformation matrix (3 x 4) pad to set the dimension (4 x 4)
	for (rowIdx = 0; rowIdx < 3; rowIdx++) {
		for (colIdx = 0; colIdx < 4; colIdx++) {
			idx = rowIdx * 4 + colIdx + 1;
			Tr_velo_cam.at<double>(rowIdx, colIdx) = std::stod(callib[5][idx]);
			Tr_imu_velo.at<double>(rowIdx, colIdx) = std::stod(callib[6][idx]);
		}
	}
	Tr_velo_cam.at<double>(3, 0) = 0.0;   Tr_imu_velo.at<double>(3, 0) = 0.0;
	Tr_velo_cam.at<double>(3, 1) = 0.0;   Tr_imu_velo.at<double>(3, 1) = 0.0;
	Tr_velo_cam.at<double>(3, 2) = 0.0;   Tr_imu_velo.at<double>(3, 2) = 0.0;
	Tr_velo_cam.at<double>(3, 3) = 1.0;   Tr_imu_velo.at<double>(3, 3) = 0.1;

	std::cout << "======================================================" << std::endl;
	// Print the callibration parameters P0
	std::cout << "PO:" << std::endl;
	for (rowIdx = 0; rowIdx < 3; rowIdx++) {
		for (colIdx = 0; colIdx < 4; colIdx++) {
			std::cout << P0.at<double>(rowIdx, colIdx) << "  ";
		}
		std::cout << std::endl;
	}
	std::cout << "======================================================" << std::endl;
	// Print the callibration parameters P1
	std::cout << "P1:" << std::endl;
	for (rowIdx = 0; rowIdx < 3; rowIdx++) {
		for (colIdx = 0; colIdx < 4; colIdx++) {
			std::cout << P1.at<double>(rowIdx, colIdx) << "  ";
		}
		std::cout << std::endl;
	}
	std::cout << "======================================================" << std::endl;
	// Print the callibration parameters P2
	std::cout << "P2:" << std::endl;
	for (rowIdx = 0; rowIdx < 3; rowIdx++) {
		for (colIdx = 0; colIdx < 4; colIdx++) {
			std::cout << P2.at<double>(rowIdx, colIdx) << "  ";
		}
		std::cout << std::endl;
	}
	std::cout << "======================================================" << std::endl;
	// Print the callibration parameters P3
	std::cout << "P3:" << std::endl;
	for (rowIdx = 0; rowIdx < 3; rowIdx++) {
		for (colIdx = 0; colIdx < 4; colIdx++) {
			std::cout << P3.at<double>(rowIdx, colIdx) << "  ";
		}
		std::cout << std::endl;
	}
	std::cout << "======================================================" << std::endl;
	// Print the callibration parameters R_rect
	std::cout << "R_rect:" << std::endl;
	for (rowIdx = 0; rowIdx < 4; rowIdx++) {
		for (colIdx = 0; colIdx < 4; colIdx++) {
			std::cout << R_rect.at<double>(rowIdx, colIdx) << "  ";
		}
		std::cout << std::endl;
	}
	std::cout << "======================================================" << std::endl;
	// Print the callibration parameters Tr_velo_cam
	std::cout << "Tr_velo_cam:" << std::endl;
	for (rowIdx = 0; rowIdx < 4; rowIdx++) {
		for (colIdx = 0; colIdx < 4; colIdx++) {
			std::cout << Tr_velo_cam.at<double>(rowIdx, colIdx) << "  ";
		}
		std::cout << std::endl;
	}
	std::cout << "======================================================" << std::endl;
}