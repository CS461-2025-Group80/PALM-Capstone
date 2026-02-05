# LIDAR Data Logger - Quick Start Guide

## Overview
ROS2 node that logs LIDAR scan data from the YDLidar to local storage for later upload to Azure.

## Prerequisites
- ROS2 Galactic
- YDLidar ROS2 driver installed at `~/myagv_ros2`
- This package in elephant-development branch

## Quick Start

### 1. Start the YDLidar driver

Open Terminal 1:
```bash
source /home/er/myagv_ros2/install/setup.bash
ros2 launch ydlidar_ros2_driver ydlidar_launch.py
```

### 2. Start the data logger

Open Terminal 2:
```bash
# Navigate to package
cd ~/palm/ros/ros-code/elephant-ros/scan_logger

# Build (only needed after code changes)
colcon build --packages-select lidar_logger

# Source both workspaces
source /home/er/myagv_ros2/install/setup.bash
source ~/palm/ros/ros-code/elephant-ros/scan_logger/install/setup.bash

# Run the logger
ros2 run lidar_logger scan_logger
```

You should see:
```
[INFO] [scan_logger]: Saving LIDAR data to: /home/er/lidar_data
[INFO] [scan_logger]: Save interval: 2.0 seconds
[INFO] [scan_logger]: Scan logger started. Listening to /scan
[INFO] [scan_logger]: Received 100 scans, saved 6 files
```

### 3. Verify data is being saved

Open Terminal 3:
```bash
# Check files
ls -lh ~/lidar_data/

# Watch files being created in real-time
watch -n 1 'ls -lht ~/lidar_data/ | head -10'

# View a sample file
cat ~/lidar_data/$(ls -t ~/lidar_data/ | head -1) | head -30
```

## Configuration

You can customize the logger with launch parameters:
```bash
ros2 launch lidar_logger scan_logger.launch.py \
    save_directory:=/custom/path \
    save_interval:=5.0 \
    file_format:=csv
```

Available parameters:
- `save_directory`: Where to save files (default: `~/lidar_data`)
- `save_interval`: Seconds between saves (default: `2.0`)
- `file_format`: `json` or `csv` (default: `json`)
- `scan_topic`: Topic name (default: `/scan`)
- `max_range_filter`: Max range in meters, 0.0 = no filter (default: `0.0`)

## File Output

Files are saved with timestamps: `YYYYMMDD_HHMMSS_microseconds.json`

Example: `20260205_153022_123456.json`

Each file contains:
- Timestamp
- Full LIDAR scan data (ranges, angles, intensities)
- Header information
- Scan parameters

## Troubleshooting

### No data received
- Check LIDAR is running: `ros2 topic list` (should see `/scan`)
- Check data is publishing: `ros2 topic echo /scan --once`

### QoS mismatch warning
- Already handled in code with BEST_EFFORT reliability

### Build errors
- Make sure you're in the package directory
- Clean build: `rm -rf build/ install/ log/` then rebuild

## Development

After making code changes:
```bash
cd ~/palm/ros/ros-code/elephant-ros/scan_logger
colcon build --packages-select lidar_logger
source install/setup.bash
```

## Next Steps
- Set up Azure upload script
- Configure automatic startup on boot
- Add data compression for storage efficiency
