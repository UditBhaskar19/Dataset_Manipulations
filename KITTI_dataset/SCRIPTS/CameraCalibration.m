% Camera Callibration %
% 1. Find chessboard corners
% 2. Compute the callibaration parameters.
% 3. Undistort the lane Images
%% ====================================================================== %
clear all;
close all;
clc;
%% ==========   VISUALIZATION FLAGS ===================================== %
DISPLAY_CHESS_BOARD_CORNERS      = false;
DISPLAY_CAMERA_EXTRINSICS        = false;
DISPLAY_UNDISTORTED_CHESS_BOARD  = false;
DISPLAY_UNDISTORTED_VIDEO_FRAMES = false;
DISPLAY_OPTICAL_FLOW             = false;
SAVE_VIDEO_FRAMES                = false;
%% ==========  CALIBRATE CAMERA AND COMPUTE CAMERA PARAMETERS =========== %
%% **********  Read the images ******************************************
dataPathRoot_chkBoard = 'F:\OpenCV\TestProgram\camera_cal';
imageType             = '.jpg';
nImgs = 20;
for idx = 1:nImgs
    imageName = strcat('calibration',num2str(idx));
    imagePath{idx} = strcat(dataPathRoot_chkBoard, '\', imageName, imageType);
end
%% ********* detect the chess-board corners *****************************
[imagePoints,boardSize,imagesUsed] = detectCheckerboardPoints(imagePath);
%% ********* display the chess-board corners ****************************
imageFileNames = imagePath(imagesUsed);
numValidImages = sum(imagesUsed);
if(DISPLAY_CHESS_BOARD_CORNERS)
figure(1)
for i = 1:numel(imageFileNames)-14
  I = imread(imageFileNames{i});
  subplot(2, 2, i);
  imshow(I);
  hold on;
  plot(imagePoints(:,1,i),imagePoints(:,2,i),'ro');
end
end
%% ===========  Generate world points =================================== %
squareSizeInMM = 35;
worldPoints = generateCheckerboardPoints(boardSize,squareSizeInMM);
%% ===========  estimate the camera parameters and visualize ============ %
[cameraParams,imagesUsed,estimationErrors] = estimateCameraParameters(imagePoints,worldPoints);
if(DISPLAY_CAMERA_EXTRINSICS)
figure(2);
showExtrinsics(cameraParams);
end
%% =========== undistort the image ====================================== %
I = imread(imageFileNames{6});
J1 = undistortImage(I,cameraParams);
%% ***********************************************************************
if(DISPLAY_UNDISTORTED_CHESS_BOARD)
figure(3); imshowpair(I,J1,'montage');
title('Original Image (left) vs. Corrected Image (right)');
end
%% ****************** READ VIDEO ************************* %
videoRoot = 'F:\OpenCV\TestProgram\camera_cal\RoadVideosUDACITY';
videoType = '.mp4';
video1 = 'project_video'; video2 = 'challenge_video'; video3 = 'harder_challenge_video';
video4 = 'solidWhiteRight' ; video5 = 'solidYellowLeft';
video = video4;
videoPath = strcat(videoRoot, '\', video, videoType);
videoFileReader = VideoReader(videoPath);
%% ************************************************************************
if(DISPLAY_UNDISTORTED_VIDEO_FRAMES)
figure(4);
title('Original Image (left) vs. Corrected Image (right)');
end

count = 0;
while hasFrame(videoFileReader)
    videoFrame = readFrame(videoFileReader);
    I = undistortImage(videoFrame,cameraParams);
    %% ======== SAVE THE FRAME FROM THE VIDEOS ========================== %
    if(SAVE_VIDEO_FRAMES)
    count = count + 1;
    writePathRoot = videoRoot;
    fileType      = '.png';
    folderName    = video;
    writePath     = strcat(writePathRoot, '\', folderName, '\', 'frame_', num2str(count), fileType);
    imwrite(I, writePath);
    end
    %% ========  for Optical flow using Farneback Method ================ %
    if(DISPLAY_OPTICAL_FLOW)
    opticFlow = opticalFlowFarneback;
    frameGray = rgb2gray(I);
    flow = estimateFlow(opticFlow,frameGray);
    
    imshow(I)
    hold on
    % Plot the flow vectors
    plot(flow,'DecimationFactor',[25 25],'ScaleFactor', 2)
    % Find the handle to the quiver object
    q = findobj(gca,'type','Quiver');
    % Change the color of the arrows to red
    q.Color = 'r';
    drawnow
    hold off
    end
    %% ================================================================= %
    if(DISPLAY_UNDISTORTED_VIDEO_FRAMES)
       imshow(I);
    end
    %imshowpair(videoFrame,I,'montage');
    %pause(1/videoFileReader.FrameRate);
    %% ==================  CANNY EDGE DETECTION =======================  %
    edgeImage = edge(rgb2gray(I),'canny');   %sobel
    imshow(edgeImage);
    %% ================== HOUGH LINES ================================= %
    Lines = 0;
end

%% ================== HOUGH LINES ================================= %
linesPath = 'F:\OpenCV\TestProgram\camera_cal\RoadVideosUDACITY\solidWhiteRight\frame_1.png';
LinesImgRGB = imread(linesPath);
LinesImgBW  = rgb2gray(LinesImgRGB);
LinesEdges  =  edge(LinesImgBW,'canny');
%[H,theta,rho] = hough(LinesEdges, 'RhoResolution', 0.5,'Theta', -90:0.5:89);
[H,theta,rho] = hough(LinesEdges);
%%
figure(10)
subplot(2,1,1);
imshow(LinesImgRGB);
title('ROAD.png');
subplot(2,1,2);
imshow(imadjust(rescale(H)),'XData',theta,'YData',rho,...
      'InitialMagnification','fit');
title('Hough transform of gantrycrane.png');
xlabel('\theta'), ylabel('\rho');
axis on, axis normal, hold on;
colormap(gca,hot);
%%
numpeaks = 50;
Peaks = houghpeaks(H,numpeaks); 
%%
figure(11)
imshow(H,[],'XData',theta,'YData',rho,'InitialMagnification','fit');
xlabel('\theta'), ylabel('\rho');
axis on, axis normal, hold on;
plot(theta(Peaks(:,2)),rho(Peaks(:,1)),'s','color','white');

%%
figure(12)
lines = houghlines(LinesEdges,theta,rho,Peaks,'FillGap',5,'MinLength',7);
figure, imshow(LinesImgRGB), hold on
max_len = 0;
for k = 1:length(lines)
   xy = [lines(k).point1; lines(k).point2];
   plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');

   % Plot beginnings and ends of lines
   plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
   plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');

   % Determine the endpoints of the longest line segment
   len = norm(lines(k).point1 - lines(k).point2);
   if ( len > max_len)
      max_len = len;
      xy_long = xy;
   end
end