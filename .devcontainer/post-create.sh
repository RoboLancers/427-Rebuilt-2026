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

cd /tmp
wget 'https://github.com/wpilibsuite/vscode-wpilib/releases/download/v2026.2.1/vscode-wpilib-2026.2.1.vsix'
# "code" CLI is not available inside the container, so unpack the VSIX manually.
# VS Code Server loads extensions from ~/.vscode-server/extensions/.
EXT_DIR="$HOME/.vscode-server/extensions/wpilibsuite.vscode-wpilib-2026.2.1"
mkdir -p "$EXT_DIR"
unzip -o /tmp/vscode-wpilib-2026.2.1.vsix 'extension/*' -d /tmp/vscode-wpilib-unpack
cp -r /tmp/vscode-wpilib-unpack/extension/. "$EXT_DIR/"
rm -rf /tmp/vscode-wpilib-unpack /tmp/vscode-wpilib-2026.2.1.vsix
cd -
# Create the WPILib home directory structure the vscode-wpilib extension expects.
# The extension checks ~/wpilib/2026/jdk/ to configure java.jdt.ls.java.home.
# We symlink to the system JDK installed by the devcontainer Java feature.
# Locate the JDK: prefer $JAVA_HOME, fall back to resolving the java binary.
if [ -n "$JAVA_HOME" ] && [ -d "$JAVA_HOME" ]; then
  WPILIB_JDK="$JAVA_HOME"
else
  WPILIB_JDK=$(readlink -f "$(which java)" | sed 's|/bin/java$||')
fi
echo "Using JDK at: $WPILIB_JDK"

# Create the WPILib home structure the vscode-wpilib extension expects.
# The extension looks for ~/wpilib/2026/jdk/ and sets java.jdt.ls.java.home from it.
mkdir -p ~/wpilib/2026
ln -sf "$WPILIB_JDK" ~/wpilib/2026/jdk
mkdir -p ~/wpilib/2026/maven  # Gradle checks this path (settings.gradle pluginManagement)

# Pre-populate the Gradle cache with WPILib and vendor JARs.
# Without this, the Java Language Server has no classpath and IntelliSense shows nothing.
chmod +x gradlew
./gradlew dependencies --no-daemon -q

echo "WPILib devcontainer setup complete"
