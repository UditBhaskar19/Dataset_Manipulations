clear all
clc

% load the lidar data
% -------------------
dataPathRoot = "F:\DATASET\KITTI\data_tracking_velodyne\training\velodyne";
scene        = "0000";
folder       = "CSV";
fileType     = ".csv";
dataPath     = strcat(dataPathRoot, "\", scene, "\", folder);
% -------------------
dataStart     = 153;
dataEnd       = 153;
nData         = dataEnd - dataStart + 1;
nDigits       = 3;
setFill       = ["00000" , "0000", "000"];
labelWidth    = setFill(3);
dataPathFull  = strcat(dataPath, "\", labelWidth, num2str(dataStart), fileType); 
DATA = load(dataPathFull);

% SOME OPERATIONS
% ---------------
X = DATA(:,1); Y = DATA(:,2); Z = DATA(:,3); INTENSITY = DATA(:,4);
Range = (X.^2 + Y.^2 + Z.^2).^0.5;

maxRange = max(Range);
minRange = min(Range);
normRange = Range./(maxRange - minRange);

maxIntensity = max(INTENSITY);
minIntensity = min(INTENSITY);
normIntensity = INTENSITY./(maxIntensity - minIntensity);

wtRange = 0.6; wtIntensity = 1 - wtRange;
normConfidence = wtRange.*normRange + wtIntensity.*normIntensity;

% Plot the data in 3D
% -------------------
[RangeSort, SortIdxRange] = sort(Range,'descend');
Xr = X(SortIdxRange); Yr = Y(SortIdxRange); Zr = Z(SortIdxRange);

[intensitySort, SortIdxIntensity] = sort(INTENSITY,'descend');
Xi = X(SortIdxIntensity); Yi = Y(SortIdxIntensity); Zi = Z(SortIdxIntensity);

[NormSort, SortIdxNorm] = sort(normConfidence,'descend');
Xn = X(SortIdxNorm); Yn = Y(SortIdxNorm); Zn = Z(SortIdxNorm);

Xpt = Xn; Ypt = Yn; Zpt = Zn;
colorMap = jet(length(X));

% y = sin([1:40]/10);
% plot(y, 'm*-');
% grid on;
% ax = gca % Get handle to current axes.
% ax.XColor = 'r'; % Red
% ax.YColor = 'b'; % Blue
% ax.GridAlpha = 0.9;  % Make grid lines less transparent.
% ax.GridColor = [0.1, 0.7, 0.2]; % Dark Green.


figure(1)
scatter3(Xpt,Ypt,Zpt, 1.5, colorMap, 'filled')
axis equal;
grid on;
set(gca,'XLim',[-70 70])
set(gca,'XTick',(-70:5:70))
set(gca,'YLim',[-70 70])
set(gca,'YTick',(-70:5:70))
set(gca,'color',[0 0 0])
xlabel('x(m)');
ylabel('y(m)');
zlabel('z(m)');
handle = gca;
handle.GridAlpha = 0.3;
handle.GridColor = [1 1 1];

% Thresholding based on x,y,z 
% -----------------------
XCropped = []; YCropped = []; ZCropped = []; ICropped = [];
minZ = -1.5; maxZ = 2.1;
minX = 25; minY = 10;
nPts = size(DATA, 1);
for idx = 1:nPts
    z = Z(idx); x = X(idx); y = Y(idx);
    check1 = z > minZ; check2 = z < maxZ;
    check3 = abs(x) < minX; check4 = abs(y) < minY;
    if(check1 && check2 && check3 && check4)
        XCropped(end+1) = X(idx);
        YCropped(end+1) = Y(idx);
        ZCropped(end+1) = Z(idx);
        ICropped(end+1) = INTENSITY(idx);
    end
end


% Plot the data in 3D
% -------------------
[intensitySort, SortIdxIntensity] = sort(ICropped,'descend');
Xcrop = XCropped(SortIdxIntensity); Ycrop = YCropped(SortIdxIntensity); Zcrop = ZCropped(SortIdxIntensity);
colorMap = jet(length(XCropped));
figure(2)
scatter3(Xcrop,Ycrop,Zcrop, 1.5, colorMap, 'filled')
grid on; axis equal;
set(gca,'XLim',[-30 30])
set(gca,'XTick',(-30:1:30))
set(gca,'YLim',[-10 10])
set(gca,'YTick',(-10:1:10))
set(gca,'color',[0 0 0])  %rgb(105,105,105)
xlabel('x(m)');
ylabel('y(m)');
zlabel('z(m)');
handle = gca;
handle.GridAlpha = 0.3;
handle.GridColor = [1 1 1];

% *********************************************** %
% ======>  LOAD & READ CAMERA IMAGE <============ %
% ----------------------------------------------- %


% % Perform Clustering 
% % ==================
% % initialization param
% % --------------------
% nPts = length(Xcrop);
% MeasurementsType    = zeros(nPts, 1);
% ClusterIDAssigned   = zeros(nPts, 1);
% ClusterMembers      = zeros(nPts, 1);
% ClusterCoreMembers  = zeros(nPts, 1);
% ClusterMemberFlag   = logical(zeros(nPts, 1));
% MeasurementsVisited = logical(zeros(nPts, 1));
% sizeNeighbour       = 1;
% sizeCluster         = 1;
% ClusterID           = 0;

% % Core Point Identification
% % -------------------------
% DistTHR = 15;
% for idx_i = 1:nPts
%     pointCount = 1;
%     for idx_j = idx_i+1:nPts
%         diff = [Xcrop(idx_i) - Xcrop(idx_j), Ycrop(idx_i) - Ycrop(idx_j), Zcrop(idx_i) - Zcrop(idx_j)]; 
%         dist = norm(diff);
%         if(dist <= DistTHR)
%             pointCount = pointCount + 1;
%         else
%             break;
%         end
%     end
%     if(pointCount == 1)
%         MeasurementsType(idx_i) = 0;  %noise point
%     elseif(pointCount == 2)
%         MeasurementsType(idx_i) = 2;  %neighbour point
%     elseif(pointCount > 2)
%         MeasurementsType(idx_i) = 1;  %core point
%     end
% end
% 
% disp('BEGIN CLUSTERING')
% 
% % Cluster formation
% % -----------------
% for idx_i = 1:nPts
%     if( ~MeasurementsVisited(idx_i) && MeasurementsType(idx_i)== 1)
%         MeasurementsVisited(idx_i)        = true;
%         ClusterMemberFlag(idx_i)          = true;
%         ClusterMembers(sizeCluster)       = idx_i;
% 	    ClusterCoreMembers(sizeNeighbour) = idx_i;
%     
%         for idx_j = 1:nPts
%             if( idx_i ~= idx_j && ~ClusterMemberFlag(idx_j))
%                 diff = [Xcrop(idx_i) - Xcrop(idx_j), Ycrop(idx_i) - Ycrop(idx_j), Zcrop(idx_i) - Zcrop(idx_j)]; 
%                 dist = norm(diff);
%                 if(dist <= DistTHR)
%                     sizeNeighbour = sizeNeighbour + 1;
% 				    sizeCluster   = sizeCluster + 1;
% 			        ClusterMemberFlag(idx_j) = true;
% 			        ClusterMembers(sizeCluster) = idx_j;
% 			        ClusterCoreMembers(sizeNeighbour) = idx_j;
%                 else
%                     break;
%                 end
%             end
%         end
%         ClusterID = ClusterID + 1;
%         ClusterIDAssigned(idx_i) = ClusterID;
%         index = 1;
%         
%         while(index <= sizeNeighbour)
%             probableCoreIdx = ClusterCoreMembers(index);
%             if(~MeasurementsVisited(probableCoreIdx))
%                 MeasurementsVisited(probableCoreIdx) = true;
%                 for idx_j = 1:nPts
%                     if( probableCoreIdx ~= idx_j && ~ClusterMemberFlag(idx_j) && MeasurementsType(idx_i) ~= 0)
%                         diff = [Xcrop(probableCoreIdx) - Xcrop(idx_j), Ycrop(probableCoreIdx) - Ycrop(idx_j), Zcrop(probableCoreIdx) - Zcrop(idx_j)]; 
%                         dist = norm(diff);
%                         if(dist <= DistTHR)
%                             if(MeasurementsType(idx_i) == 1)
%                                 sizeNeighbour = sizeNeighbour + 1;
% 				                sizeCluster   = sizeCluster + 1;
% 			                    ClusterMemberFlag(idx_j) = true;
% 			                    ClusterMembers(sizeCluster) = idx_j;
% 			                    ClusterCoreMembers(sizeNeighbour) = idx_j;
%                             elseif(MeasurementsType(idx_i) == 2)
%                                 sizeCluster = sizeCluster + 1;
% 					            ClusterMemberFlag(idx_j) = true;
% 								ClusterMembers(sizeCluster) = idx_j; 
%                             end
%                         else
%                             break;
%                         end
%                     
%                     end
%                 end
%             end
%             index = index + 1;
%             if(ClusterIDAssigned(probableCoreIdx) == 0)
% 				ClusterIDAssigned(probableCoreIdx) = ClusterID;
%             end
%         end  % end of while
%         
%         %reset all to 0
% 		ClusterMembers(:)     = 0;
% 		ClusterCoreMembers(:) = 0;
% 	    ClusterMemberFlag(:)  = 0;
% 		sizeCluster           = 1;
% 		sizeNeighbour         = 1;
%         
%     end % end of outer if condition
%     
% end % end of outer for loop


   








