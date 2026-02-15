# CONTRIBUTING.md

## Prerequisites

Before you begin, ensure you have:

- **Ubuntu machine** with ROS2 support (Ubuntu 22.04 recommended)
- **Git** installed
- **Python 3.11+**
- **ROS2** (distribution: `humble hawksbill`)

---

## Local Setup

### 1. Clone the Repository

```bash
git clone https://github.com/CS461-2025-Group80/PALM-Capstone
cd PALM-Capstone
```

---

## 2A. If Contributing Python Code, Tools, or Documentation:

Add new Python packages to `README.md` and/or create a `requirements.txt` file in your created directory.

Run linting with flake8 (see Development Workflow below).

---

## 2B. If Contributing ROS2 Code:

### Install ROS2 Dependencies

If you haven't already installed ROS2 Humble:

```bash
# Update package list
sudo apt update

# Install ROS2 Humble Desktop
sudo apt install ros-humble-desktop

# Install additional ROS2 tools
sudo apt install ros-dev-tools

# Source ROS2 environment (add this to your ~/.bashrc for persistence)
source /opt/ros/humble/setup.bash
```

### Navigate to the ROS Workspace

```bash
cd ~/ros/ros-code
```

### Available Nodes:

agv-ros/
- `lidar_logger`
- `gazebo_ros_pkgs`
- `myagv_description`
- `myagv_navigation2`
- `myagv_odometry`
- `adapter firmwareV1.2`
- `navigation2`
- `robot_localization`
- `slam_gmapping`
- `teleop_twist_keyboard`
- `vision_opencv`
- `ydlidar_ros2_driver`

turtlebot-ros/
- `rtabmap`
- `intel-realsense`

controller-ros/
- `VNC`
- `gstreamer-listener`

### Make Node Changes, Then:

**Build the Project Nodes:**

```bash
# Build specific package (example)
colcon build --packages-select lidar_logger

# Source the workspace
source install/setup.bash
```

See directories for exact node names and launch scripts.

**Verify the Node:**

```bash
# Start the node (example)
ros2 launch lidar_logger scan_logger.launch.py

# In another terminal, check if it's running
ros2 node list
ros2 topic list
```

---

## Development Workflow

### Code Quality Tools

#### Running Linters Locally

We use **flake8** for Python linting:

```bash
# Run flake8 on a specific file
flake8 path/to/your_file.py

# Run flake8 on a directory
flake8 path/to/directory/

# Run flake8 on the entire project
flake8 .
```

---

## Contribution Process

### 1. Create a Branch

Use clear, descriptive branch names that indicate the purpose of your work:

```bash
git checkout -b feature/[descriptive-feature-name]
# or
git checkout -b bugfix/[descriptive-bug-name]
# or
git checkout -b docs/[documentation-update]
```

**Examples:**
- `feature/remote-control-encryption`
- `bugfix/camera-latency-fix`
- `docs/update-setup-instructions`

### 2. Make Your Changes

- Write clean, readable code with comments
- Include **descriptive commit messages** that explain what and why:
  ```bash
  git commit -m "Add encryption to UDP controller input for remote control"
  ```
- Keep commits focused and atomic when possible

### 3. Test Your Changes

Before submitting, ensure:
- Your code passes `flake8` linting
- You've tested your changes locally with actual hardware (when applicable) or note that it needs testing
- Your changes meet the requirements in the Definition of Done (see below)

### 4. Open a Pull Request

When opening a PR, include:

- **Clear title** summarizing the change
- **Description** of what changed and why
- **Testing performed** (especially for hardware-dependent features)
- **Testing needed** (when testing isn't complete)
- **Reference to related issues** (if applicable)
- **Screenshots or logs** (if relevant)

---

## Definition of Done (DoD)

Our project has an evolving definition of done. Currently, we're focused on these major tasks:

### Current Priority: Remote Controlling Support

Your contribution is considered complete when:

**Code Quality:**
- Code follows coding standards
- Code is easily readable and well-commented
- Passes flake8 linting without errors

**Documentation:**
- New features include appropriate documentation
- README or relevant docs are updated if setup/usage changes

**Review:**
- All review feedback has been addressed

---

## Code Review Expectations

### For Contributors:
- Be open to feedback and constructive criticism
- Respond to review comments in a timely manner
- Make requested changes or discuss alternatives
- Keep discussions professional and focused on the code

### For Reviewers:
- Ensure code follows outlined standards
- Check for readability and maintainability
- Verify that changes align with the Definition of Done
- Test changes when possible (especially hardware-dependent features)
- Provide constructive, actionable feedback

---

## Reporting Bugs and Requesting Features

### Where to Report

**Issues** should be reported in [GitHub Issues](https://github.com/CS461-2025-Group80/PALM-Capstone/issues)

### Bug Report Template

When reporting a bug, please include:

- **Title:** Brief, descriptive summary
- **Description:** What happened vs. what you expected
- **Steps to Reproduce:**
  1. Step one
  2. Step two
  3. ...
- **Environment:**
  - Ubuntu version
  - ROS2 distribution
  - Hardware (if applicable)
  - Python version
- **Logs/Screenshots:** Any relevant error messages or visual evidence
- **Possible Solution:** (Optional) If you have ideas on how to fix it

---

## Getting Help

If you need assistance or have questions:

### Team Contacts

Reach out to any of the following team members:

- **fiefa@oregonstate.edu**
- **peytonju@oregonstate.edu**
- **hashbarl@oregonstate.edu**

### Discord

Ask to join our Discord server for real-time discussion and collaboration.

---

## Additional Notes

- Our project proposer has taken an organic approach with evolving requirements
- Additional tasks will be assigned after completing current major objectives
- We're not strictly bound by the Capstone project posting; requirements evolve based on team capabilities

---

## Thank You!

We appreciate your contributions to the REVOBOTS PALM project. Your work helps us learn and build something meaningful together!
