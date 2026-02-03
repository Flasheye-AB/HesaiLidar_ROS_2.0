# HesaiLidar_ROS_2.0

################################################################################
Friday, June 9th, 2023 16:45 
## version
V2.0.1

## modify
1. first update
2. fix AT128 frame segmentation bug

################################################################################
Monday, October 16th, 2023 11:00 
## version
V2.0.4

## modify
1. support ET25 

################################################################################
Wednesday, October 25th, 2023 20:00 
## version
V2.0.5

## modify
1. Fix bug in decode OT128 timestamp

################################################################################
Monday, January 15th, 2024 21:30
## version
V2.0.6

## modify
1. Add point cloud timestamp usage type configuration
2. Support P128 and XT32 to set the speed and standby through the configuration file
3. Add support for AT128 to intercept FOV display

################################################################################
Saturday April 13th, 2024 20:10:00 CST
## version
V2.0.7

## modify
1. Fix gpu compile problem
2. Resolve compile warnings, inluding cpu compile and gpu compile
3. ROS add topics, support angle correction data, ptp data, loss bag count data

################################################################################
Wednesday September 25th, 2024 11:04:33 CST
## version
V2.0.8

## modify
1. Addition of mechanical lidar photocentre correction parameters: distance_correction_lidar_type
2. Adding XT lidar point cloud S-stratification correction parameters: lidar_type
3. Add parameter to filter point cloud or fault message data for a specified port: device_udp_src_port, device_fault_port

################################################################################
Monday December 23rd, 2024 14:57:29 CST
## version
V2.0.9

## modify
1. Distance Correction and XT S Point Cloud Layering Correction No need to specify radar model, via flag position switch

################################################################################
Monday June 30rd, 2025 17:28:15
## version
2.0.10

## modify
1. Modify the units of IMU output data to m/s² and rad/s respectively
2. Use multicast_ip_address instead of group_address
3. Modify the config.yaml configuration table, configure different data sources through source_type, and fill in the corresponding configuration items
4. send_imu_ros controls whether IMU data registers callback functions and whether to publish IMU data
5. Support jazzy version, support install

################################################################################
Tuesday November 25th, 2025
## version
preliminary

## modify
1. Updated to support DoRemake and return point cloud organized in rows and columns
2. Works with updated SDK module with dense organized point cloud for OT128
3. Defaults to not use DoRemake. Set in config/config.yaml

################################################################################
February 2025 (Flasheye modifications)
## version
??

## modify
1. RemakeConfig support: Output point cloud as organized grid (height x width) instead 
   of unordered sequence (height=1). Required for range binning segmentation.
2. OT128 ring-based vertical binning: Dense 128-row grid using ring index instead of 
   sparse 320-row grid from elevation angles.
3. Optional sparse ring duplication for OT128: Fills horizontal gaps in outer rings (0-23, 88-127).
4. echo_mode_filter config option: Filter dual returns to single return mode.
   0=all returns, 1=first return only, 2=second return only.
5. Test/debug feature, requires compile time define: Subsampling support in ToRosMsg: Compile-time constants kSubsampleX/kSubsampleY 
   for reducing output resolution during testing. Set USE_SUB_SAMPLING_FOR_TEST to enable.

## config.yaml options
remake_config:
  enabled: true                    # Enable ordered grid output
  use_ring_for_vertical: true      # OT128: use ring index for vertical axis
  duplicate_sparse_rings: false     # OT128: fill gaps in sparse rings
  echo_mode_filter: 1              # 0=all, 1=first, 2=second return
