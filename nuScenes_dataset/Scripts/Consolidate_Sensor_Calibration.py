# -*- coding: utf-8 -*-
"""
Created on Fri Feb 19 17:07:40 2021

@author: Udit Bhaskar
"""


import cv2

import csv
#import argparse
import os
import os.path as osp
#from typing import Tuple

import numpy as np
#from PIL import Image
#from pyquaternion import Quaternion
from tqdm import tqdm

#from nuscenes.utils.geometry_utils import view_points, transform_matrix
from nuscenes.utils.data_classes import LidarPointCloud
from nuscenes.utils.data_classes import RadarPointCloud
from nuscenes.nuscenes import NuScenes
from nuscenes.can_bus.can_bus_api import NuScenesCanBus
from nuscenes.map_expansion.map_api import NuScenesMap

nusc_dat = NuScenes(version='v1.0-mini', dataroot='F:/nuscenes_mini/v1.0-mini_US/v1.0-mini', verbose=True)
nusc_can = NuScenesCanBus(dataroot='F:/nuscenes_mini/v1.0-mini_US/v1.0-mini')
#nusc_map = NuScenesMap(dataroot='F:/nuscenes_mini/v1.0-mini_US/v1.0-mini', map_name='singapore-onenorth')

# List the available scenes (TOTAL 10 scenes)
# ===========================================
# 1) scene-0061, Parked truck, construction, intersectio... [18-07-24 03:28:47]   19s, singapore-onenorth, #anns:4622
# 2) scene-0103, Many peds right, wait for turning car, ... [18-08-01 19:26:43]   19s, boston-seaport, #anns:2046
# 3) scene-0655, Parking lot, parked cars, jaywalker, be... [18-08-27 15:51:32]   20s, boston-seaport, #anns:2332
# 4) scene-0553, Wait at intersection, bicycle, large tr... [18-08-28 20:48:16]   20s, boston-seaport, #anns:1950
# 5) scene-0757, Arrive at busy intersection, bus, wait ... [18-08-30 19:25:08]   20s, boston-seaport, #anns:592
# 6) scene-0796, Scooter, peds on sidewalk, bus, cars, t... [18-10-02 02:52:24]   20s, singapore-queensto, #anns:708
# 7) scene-0916, Parking lot, bicycle rack, parked bicyc... [18-10-08 07:37:13]   20s, singapore-queensto, #anns:2387
# 8) scene-1077, Night, big street, bus stop, high speed... [18-11-21 11:39:27]   20s, singapore-hollandv, #anns:890
# 9) scene-1094, Night, after rain, many peds, PMD, ped ... [18-11-21 11:47:27]   19s, singapore-hollandv, #anns:1762
#10) scene-1100, Night, peds in sidewalk, peds cross cro... [18-11-21 11:49:47]   19s, singapore-hollandv, #anns:935
nusc_dat.list_scenes()

# Global Settings
# ===============
# RadarPointCloud.default_filters()
RadarPointCloud.disable_filters()
nSCENES = 10;
 
valid_channels = ['RADAR_FRONT', 
                  'RADAR_FRONT_RIGHT', 
                  'RADAR_FRONT_LEFT', 
                  'RADAR_BACK_LEFT',
                  'RADAR_BACK_RIGHT',
                  'LIDAR_TOP',
                  'CAM_FRONT', 
                  'CAM_FRONT_RIGHT', 
                  'CAM_BACK_RIGHT', 
                  'CAM_BACK', 
                  'CAM_BACK_LEFT',
                  'CAM_FRONT_LEFT']  

HEADER_RADAR  = ['x', 
                 'y', 
                 'z', 
                 'dyn_prop', 
                 'id', 
                 'rcs', 
                 'vx', 
                 'vy', 
                 'vx_comp', 
                 'vy_comp', 
                 'is_quality_valid', 
                 'ambig_state', 
                 'x_rms', 
                 'y_rms', 
                 'invalid_state', 
                 'pdh0',
                 'vx_rms',
                 'vy_rms']

HEADER_LIDAR = ['x', 'y', 'z', 'intensity']

HEADER_INSTALLATION_PARAM = ['SampleNo', 'Tx', 'Ty', 'Tz', 'Q11', 'Q12', 'Q21', 'Q22']

HEADER_INSTALLATION_PARAM_CAM = ['SampleNo', 'Tx', 'Ty', 'Tz', 
                                 'Q11', 'Q12', 'Q21', 'Q22' ,
                                 'C11', 'C12', 'C13', 'C21', 'C22', 'C23', 'C31', 'C32', 'C33']

fileType = '.csv'
Delimiter = ','
imageType = '.jpg'
FolderName = 'CALIBRATION'


# Itertate over each scene and write the sensor detections: 
# ========================================================
for sensorIdx in range(len(valid_channels)):
      channel = valid_channels[sensorIdx]
      for scene in range(nSCENES):
            # scene information (scene metadata accessed by scene number) with following attributes :
            # 1. token     : scene identifier (ID)
            # 2. log_token : 
            # 3. nbr_samples : number of samples (each sample is a collection of data from 12 sensors) 
            # 4. first_sample_token : sample ID for the first sample 
            # 5. last_sample_token  : sample ID for the last token
            # 6. name : scene name
            # 7. description : scene description
            my_scene = nusc_dat.scene[scene]
            scene_name  = my_scene['name']           #scene name
            scene_token = my_scene['token']          #scene identifier
            #scene information (scene metadata accessed by scene token: SAME as my_scene)
            scene_records = nusc_dat.get('scene', scene_token)  
            # FIRST sample information (sample metadata) with the following attributes (a sample is a record from the scene):
            # 1. token : sample ID for a sample (in case it is the first sample then same as 'first_sample_token' attribute in 'my_scene' in the above)
            # 2. timestamp : timestamp of the scene record (after SENOR SYNCHRONIZATION ???)
            # 3. prev : the previous sample ID (if null then it means the previous sample does not exist : first sample record in the scene)
            # 4. next : the next sample ID
            # 5. scene_token : scene identifier (ID) same as in 'my_scene'
            # 6. data : data identifier (ID) corresponding to different sensor modatlities
            # 7. anns : list of annotation IDs for that specific sample
            start_sample_records = nusc_dat.get('sample', my_scene['first_sample_token'])
            # detailed reference to the sensor data for a particular sensor used for data capture in the sample with the following attributes:
            # 1.  token : sensor data token for a specific sensor modality (same as 'data' in 'scene records' : see above)
            # 2.  sample_token : sample ID for a sample (same as 'token' in 'scene records')
            # 3.  ego_pose_token : ego parameters token
            # 4.  calibrated_sensor_token : sensor callibration parameters ID
            # 5.  timestamp : (sensor capture time ?????????)  
            # 6.  fileformat : specific file formats (if radar/lidar then pcd , if camera then image)
            # 7.  is_key_frame : flag that shall indicate if it is a key frame (annotated frame)
            # 8.  height   : if sensor is camera , then frame height otherwise null
            # 9.  width    : if sensor is camera , then frame width otherwise null
            # 10. filename : file name of the sensor captured data
            # 11. prev : (??????????)
            # 12. next : (??????????)
            # 13. sensor_modality : radar / camera / lidar
            # 14. channel : sensor location specifiers
            sensor_data_records  = nusc_dat.get('sample_data', start_sample_records['data'][channel])
            

            # Make list of frames
            current_sensor_data_records = sensor_data_records
            sensor_data_tokens = []
            sensor_data_tokens.append(current_sensor_data_records['token'])

            while current_sensor_data_records['next'] != '':
                  current_sensor_data_records = nusc_dat.get('sample_data', current_sensor_data_records['next'])
                  sensor_data_tokens.append(current_sensor_data_records['token'])
                    
                    
            # Out Path for CSV
            # ----------------
            out_dir = 'nuscenes_mini_analysis/v1.0-mini_US/v1.0-mini/sweeps/Experiments/' 
            out_dir2 = out_dir + scene_name + '/' + FolderName +'/'
            out_path = out_dir2 + channel + '_calib' + fileType
            if not out_dir2 == '' and not osp.isdir(out_dir2):
                    os.makedirs(out_dir2)
                    
            nEleTranslation = 3
            nEleRotation = 4
            nEleCam = 9
            nSweeps = len(sensor_data_tokens)
            nRows = nSweeps
            if(channel == 'RADAR_FRONT' or 
               channel == 'RADAR_FRONT_RIGHT' or
               channel == 'RADAR_FRONT_LEFT' or
               channel == 'RADAR_BACK_LEFT' or
               channel == 'RADAR_BACK_RIGHT' or 
               channel == 'LIDAR_TOP'):
                    nCols = nEleTranslation + nEleRotation
                    SensorParameters = np.zeros([nRows, nCols + 1], dtype = np.float64)
                    
            if(channel == 'CAM_FRONT' or
               channel == 'CAM_FRONT_RIGHT' or
               channel == 'CAM_BACK_RIGHT' or
               channel == 'CAM_BACK' or
               channel == 'CAM_BACK_LEFT' or
               channel == 'CAM_FRONT_LEFT'):
                    nCols = nEleTranslation + nEleRotation + nEleCam
                    SensorParameters = np.zeros([nRows, nCols + 1], dtype = np.float64)
            
            time = 1
            nSamples = len(sensor_data_tokens)    
            current_sensor_data_records = sensor_data_records 
            for idx in range(nSamples):
                    timeStamp   = current_sensor_data_records['timestamp']
                    sampleToken = current_sensor_data_records['sample_token']
                    ego_pose_token = current_sensor_data_records['ego_pose_token']
                    calibrated_sensor_token = current_sensor_data_records['calibrated_sensor_token']
                    filename = current_sensor_data_records['filename']
                    calibParam = nusc_dat.get('calibrated_sensor', current_sensor_data_records['calibrated_sensor_token']) # get the calibration param
                    
                    if current_sensor_data_records['next'] != '':
                           current_sensor_data_records = nusc_dat.get('sample_data', current_sensor_data_records['next'])  # next calibration param
                           
                        
                    Translation = np.reshape(calibParam['translation'], (1,3))    # 1x3 2D row vector
                    Rotation = np.reshape(calibParam['rotation'], (2,2))          # 2x2 quarternion matrix
                    Tx  = Translation[0][0];
                    Ty  = Translation[0][1];
                    Tz  = Translation[0][2];
                    q11 = Rotation[0][0]
                    q12 = Rotation[0][1]
                    q21 = Rotation[1][0]
                    q22 = Rotation[1][1]
                        
                    SensorParameters[idx][0] = time
                    SensorParameters[idx][1] = Tx
                    SensorParameters[idx][2] = Ty
                    SensorParameters[idx][3] = Tz
                    SensorParameters[idx][4] = q11
                    SensorParameters[idx][5] = q12
                    SensorParameters[idx][6] = q21
                    SensorParameters[idx][7] = q22
                        
                        
                    if(channel == 'CAM_FRONT' or
                       channel == 'CAM_FRONT_RIGHT' or
                       channel == 'CAM_BACK_RIGHT' or
                       channel == 'CAM_BACK' or
                       channel == 'CAM_BACK_LEFT' or
                       channel == 'CAM_FRONT_LEFT'):
                        
                        IntrinsicParam = np.reshape(calibParam['camera_intrinsic'], (3,3))
                        c11 = IntrinsicParam[0][0]
                        c12 = IntrinsicParam[0][1]
                        c13 = IntrinsicParam[0][2]
                        c21 = IntrinsicParam[1][0]
                        c22 = IntrinsicParam[1][1]
                        c23 = IntrinsicParam[1][2]
                        c31 = IntrinsicParam[2][0]
                        c32 = IntrinsicParam[2][1]
                        c33 = IntrinsicParam[2][2]
                        
                        SensorParameters[idx][8]  = c11
                        SensorParameters[idx][9]  = c12
                        SensorParameters[idx][10] = c13
                        SensorParameters[idx][11] = c21
                        SensorParameters[idx][12] = c22
                        SensorParameters[idx][13] = c23
                        SensorParameters[idx][14] = c31
                        SensorParameters[idx][15] = c32
                        SensorParameters[idx][16] = c33
                        
                        
                    time = time + 1
                        
            
            if(channel == 'RADAR_FRONT' or 
               channel == 'RADAR_FRONT_RIGHT' or
               channel == 'RADAR_FRONT_LEFT' or
               channel == 'RADAR_BACK_LEFT' or
               channel == 'RADAR_BACK_RIGHT' or 
               channel == 'LIDAR_TOP'):
                   HEADER_INSTALLATION = HEADER_INSTALLATION_PARAM
                   
            if(channel == 'CAM_FRONT' or
               channel == 'CAM_FRONT_RIGHT' or
               channel == 'CAM_BACK_RIGHT' or
               channel == 'CAM_BACK' or
               channel == 'CAM_BACK_LEFT' or
               channel == 'CAM_FRONT_LEFT'):
                   HEADER_INSTALLATION = HEADER_INSTALLATION_PARAM_CAM
                   
            with open(out_path, 'w', newline = '') as outFile:
                 csv_writer = csv.writer(outFile)
                 csv_writer.writerow(HEADER_INSTALLATION)
                 csv_writer.writerows(SensorParameters)
                    
                        
                    



