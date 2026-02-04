# Grid Viewer

Live range image viewer for Hesai LiDAR organized point clouds via ROS2.

## Requirements

```bash
pip install -r requirements.txt
```

ROS2 Humble must be sourced.

## Reprganize points as grid

Edit config/config.yaml to:
      remake_config:
        enabled: true                      # Enable RemakeConfig
        use_ring_for_vertical: true         # OT128: ring-based vertical binning
        duplicate_sparse_rings: true       # OT128: fill sparse ring gaps

Copy pcap and calibration files to ros2 workspace directory, and edit config to match.

## Usage

```bash
# Terminal 1: Run the Hesai ROS driver
source /opt/ros/humble/setup.bash
source <workspace>/install/local_setup.bash
ros2 launch hesai_ros_driver start.py

# Terminal 2: Run the viewer
source /opt/ros/humble/setup.bash
python3 grid_viewer.py
```

## Controls

| Key | Action |
|-----|--------|
| A / Left Arrow | Pan zoom window left |
| D / Right Arrow | Pan zoom window right |
| S | Save current frame |
| Q / ESC | Exit |
