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

# Install WPILib tools directly from frcmaven.
# GradleRIO's ToolInstallTask looks for a processstarter binary via getClass().getResourceAsStream(),
# but the wpiCppTools configuration that should provide it never resolves in this environment.
# Pre-installing the tools bypasses that broken mechanism entirely.
TOOLS_DIR="$HOME/wpilib/2026/tools"
mkdir -p "$TOOLS_DIR"
FRCMAVEN="https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/tools"
WPILIB_VER="2026.2.1"

echo "Downloading processstarter launcher..."
wget -q "$FRCMAVEN/processstarter/$WPILIB_VER/processstarter-$WPILIB_VER-linuxx86-64.zip" -O /tmp/processstarter.zip
unzip -p /tmp/processstarter.zip "linux/x86-64/processstarter" > /tmp/processstarter-bin
chmod +x /tmp/processstarter-bin
rm /tmp/processstarter.zip

# Java tools: processstarter binary (renamed to tool name) + tool JAR.
# processstarter finds the JAR by looking for <its-own-name>.jar in the same directory.
# Also create an empty <ToolName>.exe stub: GradleRIO's ToolInstallTask.getScriptFile()
# always checks for the .exe path on all platforms (including Linux). runToolUnix() uses
# the extension-less binary, so the stub is never executed — it only satisfies the check.
echo "Installing Java tools..."
# Tool name (used for file names) vs Maven artifact ID (used for download URL) differ for ShuffleBoard.
# GradleRIO WPIToolsPlugin: new WPITool(project, "<ToolName>", ..., "edu.wpi.first.tools", "<artifactId>", ...)
# Files on disk must match the tool name, not the artifact ID.
for TOOL_AND_ARTIFACT in "SmartDashboard:SmartDashboard" "ShuffleBoard:Shuffleboard" "PathWeaver:PathWeaver"; do
  TOOL="${TOOL_AND_ARTIFACT%%:*}"
  ARTIFACT="${TOOL_AND_ARTIFACT##*:}"
  wget -q "$FRCMAVEN/$ARTIFACT/$WPILIB_VER/$ARTIFACT-$WPILIB_VER-linuxx64.jar" -O "$TOOLS_DIR/$TOOL.jar"
  cp /tmp/processstarter-bin "$TOOLS_DIR/$TOOL"
  touch "$TOOLS_DIR/$TOOL.exe"
done
# RobotBuilder has no platform classifier
wget -q "$FRCMAVEN/RobotBuilder/$WPILIB_VER/RobotBuilder-$WPILIB_VER.jar" -O "$TOOLS_DIR/RobotBuilder.jar"
cp /tmp/processstarter-bin "$TOOLS_DIR/RobotBuilder"
touch "$TOOLS_DIR/RobotBuilder.exe"

rm /tmp/processstarter-bin

# Native C++ tools: self-contained binaries, extracted and renamed to PascalCase tool name.
echo "Installing native tools..."
declare -A NATIVE_TOOLS=(
  [Glass]="glass"
  [OutlineViewer]="outlineviewer"
  [SysId]="sysid"
  [DataLogTool]="datalogtool"
  [wpical]="wpical"
)
for TOOL in "${!NATIVE_TOOLS[@]}"; do
  BIN="${NATIVE_TOOLS[$TOOL]}"
  wget -q "$FRCMAVEN/$TOOL/$WPILIB_VER/$TOOL-$WPILIB_VER-linuxx86-64.zip" -O /tmp/$TOOL.zip
  unzip -p /tmp/$TOOL.zip "linux/x86-64/$BIN" > "$TOOLS_DIR/$TOOL"
  chmod +x "$TOOLS_DIR/$TOOL"
  rm /tmp/$TOOL.zip
done

echo "WPILib tools installed to $TOOLS_DIR"

# Write tools.json so GradleRIO's ToolInstallTask sees the tools as already installed.
# Without this, getExistingToolVersion() returns an empty Optional even when the binaries
# exist, causing the task to call extractAndInstall() which then crashes trying to load
# processstarter via getResourceAsStream() — a resource that never resolves in this env.
# tools.json is a JSON array of ToolConfig objects (Gson parses it as ToolConfig[]).
# Each entry needs "name" (matched against tool name) and "version" (compared to artifact version).
cat > "$TOOLS_DIR/tools.json" << EOF
[
  {"name": "SmartDashboard", "version": "$WPILIB_VER"},
  {"name": "ShuffleBoard",   "version": "$WPILIB_VER"},
  {"name": "PathWeaver",     "version": "$WPILIB_VER"},
  {"name": "RobotBuilder",   "version": "$WPILIB_VER"}
]
EOF

# Pre-populate the Gradle cache with WPILib and vendor JARs.
# Without this, the Java Language Server has no classpath and IntelliSense shows nothing.
chmod +x gradlew
./gradlew dependencies --no-daemon -q

echo "WPILib devcontainer setup complete"
