#!/bin/bash

# OpenXR and Monado Installation Script
# This script downloads and installs OpenXR-SDK and Monado from source

set -e  # Exit on error

echo "========================================"
echo "OpenXR and Monado Installation Script"
echo "========================================"

# Detect distribution
if [ -f /etc/os-release ]; then
    . /etc/os-release
    DISTRO=$ID
else
    echo "Cannot detect Linux distribution"
    exit 1
fi

echo "Detected distribution: $DISTRO"

# Create build directory
BUILD_DIR="$HOME/openxr-build"
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# Install dependencies based on distribution
echo ""
echo "Installing dependencies..."

case $DISTRO in
    ubuntu|debian|pop|linuxmint)
        sudo apt update
        sudo apt install -y \
            build-essential cmake git pkg-config \
            python3 python3-pip ninja-build \
            libeigen3-dev libvulkan-dev glslang-tools \
            libx11-dev libxxf86vm-dev libxrandr-dev libxcb-randr0-dev \
            libgl1-mesa-dev libwayland-dev wayland-protocols \
            libudev-dev libusb-1.0-0-dev libhidapi-dev \
            libsystemd-dev libuvc-dev libavcodec-dev \
            libopencv-dev libv4l-dev libcjson-dev \
            libsdl2-dev libegl1-mesa-dev doxygen graphviz
        ;;
    fedora|rhel|centos)
        sudo dnf install -y \
            gcc gcc-c++ cmake git pkg-config \
            python3 python3-pip ninja-build \
            eigen3-devel vulkan-loader-devel glslang \
            libX11-devel libXxf86vm-devel libXrandr-devel libxcb-devel \
            mesa-libGL-devel wayland-devel wayland-protocols-devel \
            systemd-devel libusb1-devel hidapi-devel \
            libuvc-devel opencv-devel libv4l-devel cjson-devel \
            SDL2-devel mesa-libEGL-devel doxygen graphviz
        ;;
    arch|manjaro)
        sudo pacman -Sy --noconfirm \
            base-devel cmake git pkg-config \
            python python-pip ninja \
            eigen vulkan-icd-loader glslang \
            libx11 libxxf86vm libxrandr libxcb \
            mesa wayland wayland-protocols \
            systemd libusb hidapi \
            opencv v4l-utils cjson \
            sdl2 doxygen graphviz
        ;;
    *)
        echo "Unsupported distribution: $DISTRO"
        echo "Please install dependencies manually"
        exit 1
        ;;
esac

echo ""
echo "========================================"
echo "Building OpenXR-SDK"
echo "========================================"

# Clone OpenXR-SDK
if [ -d "OpenXR-SDK" ]; then
    echo "OpenXR-SDK directory exists, pulling latest..."
    cd OpenXR-SDK
    git pull
else
    echo "Cloning OpenXR-SDK..."
    git clone https://github.com/KhronosGroup/OpenXR-SDK.git
    cd OpenXR-SDK
fi

# Build OpenXR-SDK
mkdir -p build
cd build
cmake .. -GNinja -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
ninja
sudo ninja install

echo "OpenXR-SDK installed successfully!"

cd "$BUILD_DIR"

echo ""
echo "========================================"
echo "Building Monado"
echo "========================================"

# Clone Monado
if [ -d "monado" ]; then
    echo "Monado directory exists, pulling latest..."
    cd monado
    git pull
else
    echo "Cloning Monado..."
    git clone https://gitlab.freedesktop.org/monado/monado.git
    cd monado
fi

# Build Monado
mkdir -p build
cd build
cmake .. -GNinja -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
ninja
sudo ninja install

echo "Monado installed successfully!"

echo ""
echo "========================================"
echo "Setting up Monado as default OpenXR runtime"
echo "========================================"

# Set Monado as active runtime
mkdir -p ~/.config/openxr/1
cat > ~/.config/openxr/1/active_runtime.json << EOF
{
    "file_format_version": "1.0.0",
    "runtime": {
        "library_path": "/usr/local/lib/libopenxr_monado.so"
    }
}
EOF

echo ""
echo "========================================"
echo "Installation Complete!"
echo "========================================"
echo ""
echo "OpenXR-SDK and Monado have been installed to /usr/local"
echo ""
echo "Test your installation with:"
echo "  - openxr_runtime_list    # List available runtimes"
echo "  - monado-cli probe       # Check for VR devices"
echo "  - monado-service         # Start Monado service"
echo ""
echo "Build directory: $BUILD_DIR"
echo "========================================"