#!/bin/bash
set -e

trap 'echo "Error occurred. Installation failed."; exit 1' ERR

echo "========================================"
echo " Installing Monado OpenXR Runtime"
echo "========================================"

# Update system
echo "[1/5] Updating package lists..."
sudo apt update

# Install required dependencies
echo "[2/5] Installing dependencies..."
sudo apt install -y \
    build-essential cmake git meson ninja-build python3-pip \
    libudev-dev libx11-dev libxext-dev libxrandr-dev libxinerama-dev \
    libxcursor-dev libvulkan-dev vulkan-validationlayers-dev \
    libgl1-mesa-dev libegl1-mesa-dev libwayland-dev wayland-protocols \
    libgbm-dev libusb-1.0-0-dev pkg-config

# Clone or update Monado repo
echo "[3/5] Setting up Monado repository..."
if [ ! -d "monado" ]; then
    git clone https://gitlab.freedesktop.org/monado/monado.git
    cd monado
else
    echo "Monado directory already exists. Updating..."
    cd monado
    git pull
fi

# Build Monado
echo "[4/5] Building Monado..."
rm -rf build  # Clean any old build
meson setup build --prefix=/usr
ninja -C build

# Install Monado
echo "[5/5] Installing Monado..."
sudo ninja -C build install
sudo ldconfig

echo "========================================"
echo " Monado installation complete!"
echo "========================================"
echo
echo "To use Monado as your OpenXR runtime, run:"
echo "    export XR_RUNTIME_JSON=/usr/share/openxr/1/openxr_monado.json"
echo
echo "Or permanently add it to ~/.bashrc:"
echo '    echo "export XR_RUNTIME_JSON=/usr/share/openxr/1/openxr_monado.json" >> ~/.bashrc'
echo
echo "You can start the Monado service with:"
echo "    monado-service"
echo