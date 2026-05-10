# SETUP INSTRUCTIONS

## File Structure Setup

After downloading all files, organize them into this structure:

```
lidar_logger/                    # Create this directory
├── CMakeLists.txt               # Downloaded file
├── package.xml                  # Downloaded file
├── setup.py                     # Downloaded file
├── setup.cfg                    # Downloaded file
├── README.md                    # Downloaded file
├── lidar_logger/                # Create this subdirectory
│   ├── __init__.py             # Downloaded file
│   └── scan_logger.py          # Downloaded file
├── launch/                      # Create this subdirectory
│   └── scan_logger.launch.py  # Downloaded file
└── resource/                    # Create this subdirectory
    └── lidar_logger            # Downloaded file (empty marker file)
```

## Quick Setup Steps

### 1. Create Directory Structure
```bash
# Create the main package directory
mkdir -p ~/Downloads/lidar_logger

# Create subdirectories
mkdir -p ~/Downloads/lidar_logger/lidar_logger
mkdir -p ~/Downloads/lidar_logger/launch
mkdir -p ~/Downloads/lidar_logger/resource
```

### 2. Move Files to Correct Locations
```bash
cd ~/Downloads

# Move main config files (keep in root)
mv CMakeLists.txt lidar_logger/
mv package.xml lidar_logger/
mv setup.py lidar_logger/
mv setup.cfg lidar_logger/
mv README.md lidar_logger/

# Move Python package files
mv __init__.py lidar_logger/lidar_logger/
mv scan_logger.py lidar_logger/lidar_logger/

# Move launch file
mv scan_logger.launch.py lidar_logger/launch/

# Move resource marker
mv lidar_logger_resource lidar_logger/resource/lidar_logger
```

### 3. Copy to ROS2 Workspace
```bash
# Copy entire package to your ROS2 workspace
cp -r ~/Downloads/lidar_logger ~/ros2_ws/src/
```

### 4. Build Package
```bash
cd ~/ros2_ws
colcon build --packages-select lidar_logger
source install/setup.bash
```

### 5. Test Run
```bash
# Start the node
ros2 launch lidar_logger scan_logger.launch.py

# In another terminal, check if it's running
ros2 node list
ros2 topic list
```

## Quick Test (if /scan is not available)
If you don't have a robot connected, you can test with a dummy scan publisher:

```bash
# Terminal 1: Start the logger
ros2 launch lidar_logger scan_logger.launch.py

# Terminal 2: Publish test data (requires sensor_msgs)
ros2 topic pub /scan sensor_msgs/msg/LaserScan "{header: {frame_id: 'laser'}, angle_min: -1.57, angle_max: 1.57, angle_increment: 0.017, time_increment: 0.0, scan_time: 0.1, range_min: 0.1, range_max: 10.0, ranges: [1.0, 2.0, 3.0, 4.0, 5.0]}"
```

Then check that files are being created:
```bash
ls -lh ~/lidar_data/
```

## Customization Examples

**Save to different directory:**
```bash
ros2 launch lidar_logger scan_logger.launch.py save_directory:=/data/robot_scans
```

**Save less frequently (every 5 seconds):**
```bash
ros2 launch lidar_logger scan_logger.launch.py save_interval:=5.0
```

**Use CSV format:**
```bash
ros2 launch lidar_logger scan_logger.launch.py file_format:=csv
```

**All custom parameters:**
```bash
ros2 launch lidar_logger scan_logger.launch.py \
    save_directory:=/data/robot_scans \
    save_interval:=5.0 \
    file_format:=csv \
    max_range_filter:=10.0
```

## Verify Installation

After building, verify the package is installed:
```bash
ros2 pkg list | grep lidar_logger
ros2 pkg executables lidar_logger
```

You should see:
```
lidar_logger
lidar_logger scan_logger
```
