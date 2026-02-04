# ROS1 Docker Testing

To test in the flasheye ROS1 docker. I used PR-1062 and named it flasheye-hesai

## Container

    flasheye-hesai ( PR-1062)

## Copy and Build

From host:

    git clone --recurse-submodules git@github.com:flasheye/Hesai.git
    cd Hesai
    sudo docker cp HesaiLidar_ROS_2.0 flasheye-hesai:/tmp/
    sudo docker cp "OT128/log.pcap" flasheye-hesai:/tmp/ot128.pcap
    sudo docker cp "OT128/OT128_Angle-Correction-File.csv" flasheye-hesai:/tmp/ot128_correction.csv

Inside docker:

    sudo docker exec -it flasheye-hesai bash
    cp -r /tmp/HesaiLidar_ROS_2.0 /tmp/hesai_ws/src/
    cd /tmp/hesai_ws
    source /opt/ros/noetic/setup.bash
    catkin_make -j4
    source devel/setup.bash

## Configuration (inside docker)

    nano /tmp/hesai_ws/src/HesaiLidar_ROS_2.0/config/config.yaml

Key settings:
- `pcap_path`: `/tmp/ot128.pcap`
- `correction_file_path`: `/tmp/ot128_correction.csv`
- `ros_send_point_cloud_topic`: `/areplay/hesai/pandar`

RemakeConfig (for ordered grid output):
- `enabled`: `true`
- `use_ring_for_vertical`: `true`
- `duplicate_sparse_rings`: `false`
- `echo_mode_filter`: `1` (first return only) 

## Run

    rosrun hesai_ros_driver hesai_ros_driver_node
