#!/bin/bash
set -e

# Install system dependencies for WPILib simulation
apt-get update && apt-get install -y \
  build-essential \
  cmake \
  pkg-config \
  libgl1-mesa-glx \
  libgl1-mesa-dev \
  libxrender1 \
  libxrandr2 \
  libxinerama1 \
  libxi6 \
  libxext6 \
  libx11-dev \
  xauth \
  x11-apps \
  wget

# Download and install WPILib VSCode extension
cd /tmp
wget 'https://github.com/wpilibsuite/vscode-wpilib/releases/download/v2026.2.1/vscode-wpilib-2026.2.1.vsix'
code --install-extension /tmp/vscode-wpilib-2026.2.1.vsix || true
cd -

echo "WPILib devcontainer setup complete"
