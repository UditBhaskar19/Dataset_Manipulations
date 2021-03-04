# -*- coding: utf-8 -*-
"""
Created on Fri Feb 19 12:20:26 2021

@author: Udit
"""

import csv
import matplotlib.pyplot as plt
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
#RadarPointCloud.default_filters()
RadarPointCloud.disable_filters()
nSCENES = 10;
valid_channels = ['LIDAR_TOP', 
                  'RADAR_FRONT', 
                  'RADAR_FRONT_RIGHT', 
                  'RADAR_FRONT_LEFT', 
                  'RADAR_BACK_LEFT',
                  'RADAR_BACK_RIGHT']  
HEADER = ['x', 
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

channel = 'RADAR_FRONT'
fileType = '.csv'

scene = 1
my_scene    = nusc_dat.scene[scene]
scene_name  = my_scene['name']
scene_token = my_scene['token']
scene_records = nusc_dat.get('scene', scene_token);
start_sample_records = nusc_dat.get('sample', scene_records['first_sample_token'])
sensor_data_records  = nusc_dat.get('sample_data', start_sample_records['data'][channel])

# Make list of frames
current_sensor_data_records = sensor_data_records
sensor_data_tokens = []
sensor_data_tokens.append(current_sensor_data_records['token'])
    
count = 1;
while current_sensor_data_records['next'] != '':
      current_sensor_data_records = nusc_dat.get('sample_data', current_sensor_data_records['next'])
      sensor_data_tokens.append(current_sensor_data_records['token'])
      count = count + 1;

count2 = 0;      
for sensor_data_tokens in tqdm(sensor_data_tokens):
      records = nusc_dat.get('sample_data', sensor_data_tokens)
      sample_records = nusc_dat.get('sample', records['sample_token'])
      #radar_token = records['token']
      token = sensor_data_records['token']
      sensor_records = nusc_dat.get('sample_data', token)
        
      # Lidar or Radar Sensor Data
      # ==========================
      if(channel == 'RADAR_FRONT' or 
         channel == 'RADAR_FRONT_RIGHT' or
         channel == 'RADAR_FRONT_LEFT' or
         channel == 'RADAR_BACK_LEFT' or
         channel == 'RADAR_BACK_RIGHT'):
            pc = RadarPointCloud.from_file(osp.join(nusc_dat.dataroot, sensor_records['filename']))
      elif(channel == 'LIDAR_TOP'):
            pc = LidarPointCloud.from_file(osp.join(nusc_dat.dataroot, sensor_records['filename']))
     
      dim  = pc.points.shape[0]
      nPts = pc.points.shape[1]
      DATA = pc.points
      DATATranspose = DATA.transpose();
      count2 = count2 + 1;
      
########## CAN data ###################
      
      
# ================= OVERVIEW ===========================
# Let us get an overview of all the CAN bus messages and 
# some basic statistics (min, max, mean, stdev, etc.). 
# We will pick an arbitrary scene for that.
# nusc_can.print_all_message_stats(scene_name)
# ==========================================

# =========== VISUALIZATION =================
# plot the values in a CAN bus message over time.
# As an example let us pick the steering angle feedback message 
# and the key called "value" as described in the README. The plot below shows the steering angle.
# It seems like the scene starts with a strong left turn and then continues more or less straight.
# message_name = 'steeranglefeedback'
# key_name = 'value'
# nusc_can.plot_message_data(scene_name, message_name, key_name)

# If the data we want to plot is multi-dimensional, we need to provide an additional argument to select the dimension.
# Here we plot the acceleration along the lateral dimension (y-axis). We can see that initially this acceleration is higher.
# message_name = 'pose'
# key_name = 'accel'
#nusc_can.plot_message_data(scene_name, message_name, key_name, dimension=1)

# ==========> ERROR HANDLING <===============
# some scenes are not well aligned with the baseline route. 
# This can be due to diversions or because the human driver was not following a route.
# We compute all misaligned routes by checking if each ego pose has a baseline route within 5m.
#print(nusc_can.list_misaligned_routes())
#print(nusc_can.can_blacklist)
# ===========================================





# =========================================================================================================
mssg1 = 'ms_imu'                   # Frequency: 100Hz
mssg1_keyName1 = 'linear_accel'    # dimension:3, Acceleration vector (x, y, z) in the IMU frame in m/s^2
mssg1_keyName2 = 'q'               # dimension:4, Quaternion that transforms from IMU coordinates to a fixed reference frame. The yaw of this reference frame is arbitrary, determined by the IMU. However, the x-y plane of the reference frame is perpendicular to gravity, and z points up
mssg1_keyName3 = 'rotation_rate'   # dimension:3, Angular velocity in rad/s around the x, y, and z axes, respectively, in the IMU coordinate frame
#nusc_can.plot_message_data(scene_name, mssg1, mssg1_keyName3, dimension=1)
mesages_ms_imu      = nusc_can.get_messages(scene_name, mssg1)
length_ms_imu_data  = len(mesages_ms_imu)
ms_imu_data_key1    = np.array([m[mssg1_keyName1] for m in mesages_ms_imu])
ms_imu_data_key2    = np.array([m[mssg1_keyName2] for m in mesages_ms_imu])
ms_imu_data_key3    = np.array([m[mssg1_keyName3] for m in mesages_ms_imu])
ms_imu_utimes       = np.array([m['utime'] for m in mesages_ms_imu])
# =========================================================================================================



# ==========================================================================================================
mssg2 = 'pose'                     # The current pose of the ego vehicle, sampled at 50Hz.
mssg2_keyName1 = 'accel'           # dimension:3, Acceleration vector in the ego vehicle frame in m/s^2 
mssg2_keyName2 = 'orientation'     # dimension:4, The rotation vector in the ego vehicle frame
mssg2_keyName3 = 'pos'             # dimension:3, The position (x, y, z) in meters in the global frame. This is identical to the nuScenes ego pose, but sampled at a higher frequency.
mssg2_keyName4 = 'rotation_rate'   # dimension:3, The angular velocity vector of the vehicle in rad/s. This is expressed in the ego vehicle frame.
mssg2_keyName5 = 'vel'             # dimension:3, The velocity in m/s, expressed in the ego vehicle frame
#nusc_can.plot_message_data(scene_name, mssg2, mssg2_keyName1, dimension=1)
mesages_pose     = nusc_can.get_messages(scene_name, mssg2)
pose_data        = len(mesages_pose)
pose_data_key1   = np.array([m[mssg2_keyName1] for m in mesages_pose])
pose_data_key2   = np.array([m[mssg2_keyName2] for m in mesages_pose])
pose_data_key3   = np.array([m[mssg2_keyName3] for m in mesages_pose])
pose_data_key4   = np.array([m[mssg2_keyName4] for m in mesages_pose])
pose_data_key5   = np.array([m[mssg2_keyName5] for m in mesages_pose])
pose_utimes      = np.array([m['utime'] for m in mesages_pose])
# =======================================================================================================



# =======================================================================================================
mssg3 = 'steeranglefeedback'       # Frequency: 100Hz
mssg3_keyName1 = 'value'           # dimension:1, Steering angle feedback in radians in range [-7.7, 6.3]. 
                                   # 0 indicates no steering, positive values indicate right turns, negative values left turns
# nusc_can.plot_message_data(scene_name, mssg3, mssg3_keyName1)
mesages_steeranglefeedback = nusc_can.get_messages(scene_name, mssg3)
length_steerfdbk_data      = len(mesages_steeranglefeedback)
steerfdbk_data_key1        = np.array([m[mssg3_keyName1] for m in mesages_steeranglefeedback])
steerfdbk_utimes           = np.array([m['utime'] for m in mesages_steeranglefeedback]) 
# =======================================================================================================

                            
# ========================================================================================================
mssg4 = 'vehicle_monitor'               # Frequency: 2Hz
mssg4_keyName1  = 'available_distance'  # dimension:1, Available vehicle range given the current battery level in kilometers
mssg4_keyName2  = 'battery_level'       # dimension:1, Current battery level in range [0, 100]
mssg4_keyName3  = 'brake'               # dimension:1, Braking pressure in bar. An integer in range [0, 126].
mssg4_keyName4  = 'brake_switch'        # dimension:1, Brake switch as an integer, 1 (pedal not pressed), 2 (pedal pressed) or 3 (pedal confirmed pressed).
mssg4_keyName5  = 'gear_position'       # dimension:1, The gear position as an integer, typically 0 (parked) or 7 (driving).
mssg4_keyName6  = 'left_signal'         # dimension:1, Left turning signal as an integer, 0 (inactive) or 1 (active).
mssg4_keyName7  = 'rear_left_rpm'       # dimension:1, Rear left brake speed in revolutions per minute.
mssg4_keyName8  = 'rear_right_rpm'      # dimension:1, Rear right brake speed in revolutions per minute.
mssg4_keyName9  = 'right_signal'        # dimension:1, Right turning signal as an integer, 0 (inactive) or 1 (active).
mssg4_keyName10 = 'steering'            # dimension:1, Steering angle in degrees at a resolution of 0.1 in range [-780, 779.9].
mssg4_keyName11 = 'steering_speed'      # dimension:1, Steering speed in degrees per second in range [-465, 393].
mssg4_keyName12 = 'throttle'            # dimension:1, Throttle pedal position as an integer in range [0, 1000].
mssg4_keyName13 = 'vehicle_speed'       # dimension:1, Vehicle speed in km/h at a resolution of 0.01.
mssg4_keyName14 = 'yaw_rate'            # dimension:1, Yaw turning rate in degrees per second at a resolution of 0.1.
mesages_vehicle_monitor     = nusc_can.get_messages(scene_name, mssg4)
length_vehicle_monitor_data = len(mesages_vehicle_monitor)
vehicle_moitor_data_key1    = np.array([m[mssg4_keyName1] for m in mesages_vehicle_monitor])
vehicle_moitor_data_key2    = np.array([m[mssg4_keyName2] for m in mesages_vehicle_monitor])
vehicle_moitor_data_key3    = np.array([m[mssg4_keyName3] for m in mesages_vehicle_monitor])
vehicle_moitor_data_key4    = np.array([m[mssg4_keyName4] for m in mesages_vehicle_monitor])
vehicle_moitor_data_key5    = np.array([m[mssg4_keyName5] for m in mesages_vehicle_monitor])
vehicle_moitor_data_key6    = np.array([m[mssg4_keyName6] for m in mesages_vehicle_monitor])
vehicle_moitor_data_key7    = np.array([m[mssg4_keyName7] for m in mesages_vehicle_monitor])
vehicle_moitor_data_key8    = np.array([m[mssg4_keyName8] for m in mesages_vehicle_monitor])
vehicle_moitor_data_key9    = np.array([m[mssg4_keyName9] for m in mesages_vehicle_monitor])
vehicle_moitor_data_key10   = np.array([m[mssg4_keyName10] for m in mesages_vehicle_monitor])
vehicle_moitor_data_key11   = np.array([m[mssg4_keyName11] for m in mesages_vehicle_monitor])
vehicle_moitor_data_key12   = np.array([m[mssg4_keyName12] for m in mesages_vehicle_monitor])
vehicle_moitor_data_key13   = np.array([m[mssg4_keyName13] for m in mesages_vehicle_monitor])
vehicle_moitor_data_key14   = np.array([m[mssg4_keyName14] for m in mesages_vehicle_monitor])
vehicle_moitor_utimes       = np.array([m['utime'] for m in mesages_vehicle_monitor])
# ============================================================================================================


# ============================================================================================================
mssg5 = 'zoesensors'                    # Frequency: 794-973Hz
mssg5_keyName1  = 'brake_sensor'        # dimension:1, Vehicle break sensor in range [0.375,0.411]. High values indicate breaking
mssg5_keyName2  = 'steering_sensor'     # dimension:1, Vehicle steering sensor. Same as vehicle_monitor.steering
mssg5_keyName3  = 'throttle_sensor'     # dimension:1, Vehicle throttle sensor. Same as vehicle_monitor.throttle
mesages_zoesensors = nusc_can.get_messages(scene_name, mssg5)
length_zoesensors_data = len(mesages_zoesensors)
zoesensors_data_key1 = np.array([m[mssg5_keyName1] for m in mesages_zoesensors])
zoesensors_data_key2 = np.array([m[mssg5_keyName2] for m in mesages_zoesensors])
zoesensors_data_key3 = np.array([m[mssg5_keyName3] for m in mesages_zoesensors])
zoesensors_utime     = np.array([m['utime'] for m in mesages_zoesensors])
# =============================================================================================================


# =============================================================================================================
mssg6 = 'zoe_veh_info'                       # Frequency: 100Hz
mssg6_keyName1  = 'FL_wheel_speed'           # dimension:1, Front left wheel speed. The unit is rounds per minute with a resolution of 0.0417rpm
mssg6_keyName2  = 'FR_wheel_speed'           # dimension:1, Front right wheel speed. The unit is rounds per minute with a resolution of 0.0417rpm
mssg6_keyName3  = 'RL_wheel_speed'           # dimension:1, Rear left wheel speed. The unit is rounds per minute with a resolution of 0.0417rpm
mssg6_keyName4  = 'RR_wheel_speed'           # dimension:1, Rear right wheel speed. The unit is rounds per minute with a resolution of 0.0417rpm
mssg6_keyName5  = 'left_solar'               # dimension:1, Zoe vehicle left solar sensor value as an integer
mssg6_keyName6  = 'longitudinal_accel'       # dimension:1, Longitudinal acceleration in meters per second squared at a resolution of 0.05
mssg6_keyName7  = 'meanEffTorque'            # dimension:1, Actual torque delivered by the engine in Newton meters at a resolution of 0.5. Values in range [-400, 1647], offset by -400
mssg6_keyName8  = 'odom'                     # dimension:1, Odometry distance travelled modulo vehicle circumference. Values are in centimeters in range [0, 124]. Note that due to the low sampling frequency these values are only useful at low speeds
mssg6_keyName9  = 'odom_speed'               # dimension:1, Vehicle speed in km/h. Values in range [0, 60]. For a higher sampling rate refer to the pose.vel message
mssg6_keyName10 = 'pedal_cc'                 # dimension:1, Throttle value. Values in range [0, 1000]
mssg6_keyName11 = 'regen'                    # dimension:1, Coasting throttle. Values in range [0, 100]
mssg6_keyName12 = 'requestedTorqueAfterProc' # dimension:1, Input torque requested in Newton meters at a resolution of 0.5. Values in range [-400, 1647], offset by -400
mssg6_keyName13 = 'right_solar'              # dimension:1, Zoe vehicle right solar sensor value as an integer
mssg6_keyName14 = 'steer_corrected'          # dimension:1, Steering angle (steer_raw) corrected by an offset (steer_offset_can)
mssg6_keyName15 = 'steer_offset_can'         # dimension:1, Steering angle offset in degrees, typically -12.6
mssg6_keyName16 = 'steer_raw'                # dimension:1, Raw steering angle in degrees
mssg6_keyName17 = 'transversal_accel'        # dimension:1, Transversal acceleration in g at a resolution of 0.004
mesages_zoe_veh_info     = nusc_can.get_messages(scene_name, mssg6)
length_zoe_veh_info_data = len(mesages_zoe_veh_info)
veh_info_data_key1       = np.array([m[mssg6_keyName1] for m in mesages_zoe_veh_info])
veh_info_data_key2       = np.array([m[mssg6_keyName2] for m in mesages_zoe_veh_info])
veh_info_data_key3       = np.array([m[mssg6_keyName3] for m in mesages_zoe_veh_info])
veh_info_data_key4       = np.array([m[mssg6_keyName4] for m in mesages_zoe_veh_info])
veh_info_data_key5       = np.array([m[mssg6_keyName5] for m in mesages_zoe_veh_info])
veh_info_data_key6       = np.array([m[mssg6_keyName6] for m in mesages_zoe_veh_info])
veh_info_data_key7       = np.array([m[mssg6_keyName7] for m in mesages_zoe_veh_info])
veh_info_data_key8       = np.array([m[mssg6_keyName8] for m in mesages_zoe_veh_info])
veh_info_data_key9       = np.array([m[mssg6_keyName9] for m in mesages_zoe_veh_info])
veh_info_data_key10      = np.array([m[mssg6_keyName10] for m in mesages_zoe_veh_info])
veh_info_data_key11      = np.array([m[mssg6_keyName11] for m in mesages_zoe_veh_info])
veh_info_data_key12      = np.array([m[mssg6_keyName12] for m in mesages_zoe_veh_info])
veh_info_data_key13      = np.array([m[mssg6_keyName13] for m in mesages_zoe_veh_info])
veh_info_data_key14      = np.array([m[mssg6_keyName14] for m in mesages_zoe_veh_info])
veh_info_data_key15      = np.array([m[mssg6_keyName15] for m in mesages_zoe_veh_info])
veh_info_data_key16      = np.array([m[mssg6_keyName16] for m in mesages_zoe_veh_info])
veh_info_data_key17      = np.array([m[mssg6_keyName17] for m in mesages_zoe_veh_info])
veh_info_utime           = np.array([m['utime'] for m in mesages_zoe_veh_info])
# =====================================================================================================================
