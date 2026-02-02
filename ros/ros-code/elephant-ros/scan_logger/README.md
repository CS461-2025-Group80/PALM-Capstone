# LIDAR Logger - ROS2 Package

A ROS2 package for logging LIDAR scan data from `/scan` topic to local storage at a configurable rate.

## Features

- Subscribes to `/scan` topic
- Saves data at configurable intervals (default: 2 seconds) to prevent system overload
- Supports JSON and CSV file formats
- Configurable save directory
- Optional range filtering to reduce file size
- Includes launch file for easy parameter configuration

## Package Contents

```
lidar_logger/
├── lidar_logger/
│   ├── __init__.py
│   └── scan_logger.py          # Main node implementation
├── launch/
│   └── scan_logger.launch.py   # Launch file
├── resource/
│   └── lidar_logger            # Resource marker file
├── package.xml
├── setup.py
├── setup.cfg
├── CMakeLists.txt
└── README.md
```

## Installation

1. Copy this package to your ROS2 workspace src directory:
   ```bash
   cd ~/ros2_ws/src
   cp -r /path/to/lidar_logger .
   ```

2. Build the package:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select lidar_logger
   ```

3. Source the workspace:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

## Usage

### Method 1: Using Launch File (Recommended)

Launch with default parameters:
```bash
ros2 launch lidar_logger scan_logger.launch.py
```

Launch with custom parameters:
```bash
ros2 launch lidar_logger scan_logger.launch.py \
    save_directory:=/home/user/my_lidar_data \
    save_interval:=5.0 \
    file_format:=csv
```

### Method 2: Running Node Directly

```bash
ros2 run lidar_logger scan_logger
```

With custom parameters:
```bash
ros2 run lidar_logger scan_logger \
    --ros-args \
    -p save_directory:=/home/user/lidar_data \
    -p save_interval:=3.0 \
    -p file_format:=json
```

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `save_directory` | string | `~/lidar_data` | Directory where scan files will be saved |
| `save_interval` | float | `2.0` | Time in seconds between saving scans |
| `scan_topic` | string | `/scan` | Name of the LaserScan topic to subscribe to |
| `file_format` | string | `json` | File format: `json` or `csv` |
| `max_range_filter` | float | `0.0` | Maximum range to save (0.0 = no filtering) |

## File Formats

### JSON Format
Each file contains:
- Timestamp
- Scan header information
- Angle configuration (min, max, increment)
- Range and intensity arrays
- Metadata (scan_time, time_increment, etc.)

Example filename: `20240215_143022_123456.json`

### CSV Format
Each file contains rows with:
- timestamp, angle, range, intensity

Example filename: `20240215_143022_123456.csv`

## Monitoring

The node logs information every 100 scans:
```
[scan_logger]: Received 500 scans, saved 25 files
```

## Future Enhancements (Azure Upload)

The package is designed to facilitate future Azure cloud upload:
- Files are saved with timestamped filenames for easy batch processing
- Structured format (JSON/CSV) for cloud storage
- Can add Azure Blob Storage upload functionality in a separate node or script
