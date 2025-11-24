#!/bin/bash

set -e

echo "========================================"
echo " Installing OpenXR SDK (Khronos)"
echo "========================================"

# Update package lists
echo "[1/4] Updating packages..."
sudo apt update

# Install dependencies required to build OpenXR-SDK
echo "[2/4] Installing dependencies..."
sudo apt install -y \
    git cmake build-essential python3 python3-pip \
    libx11-dev libxcb1-dev libxcb-ewmh-dev libxcb-icccm4-dev \
    libxcb-keysyms1-dev libxcb-randr0-dev libxcb-shape0-dev \
    libxcb-sync-dev libxcb-xfixes0-dev libxcb-xinerama0-dev \
    libwayland-dev wayland-protocols libvulkan-dev

# Clone OpenXR-SDK from Khronos
echo "[3/4] Cloning OpenXR-SDK repository..."
if [ ! -d "OpenXR-SDK" ]; then
    git clone https://github.com/KhronosGroup/OpenXR-SDK.git
else
    echo "OpenXR-SDK directory already exists. Skipping clone."
fi

# Build the SDK
echo "[4/4] Building and installing OpenXR-SDK..."
cd OpenXR-SDK
cmake -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_LOADER=ON -DBUILD_ALL_EXTENSIONS=ON
cmake --build build -j$(nproc)

# Install to /usr/local
sudo cmake --install build

echo "========================================"
echo " OpenXR SDK installation complete!"
echo "========================================"

echo
echo "This installed:"
echo " - OpenXR headers (/usr/local/include/openxr)"
echo " - OpenXR loader (/usr/local/lib/libopenxr_loader.so)"
echo " - OpenXR JSON schemas"
echo " - OpenXR validation layers (optional)"
echo
echo "To check installation:"
echo "    xr_loader_info"
echo
echo "To build apps, include:"
echo "    #include <openxr/openxr.h>"
echo "Compile with:"
echo "    -lxr_loader"
echo
echo "Remember: OpenXR still needs a runtime like Monado, SteamVR, or Oculus runtime."
echo
