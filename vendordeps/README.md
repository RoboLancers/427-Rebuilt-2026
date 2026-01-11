# Vendor Dependencies

## What are vendordeps?

**Vendor dependencies (vendordeps)** are JSON configuration files that tell Gradle where to download third-party libraries that aren't included in the base WPILib installation.

### Purpose:

- **Define external libraries**: Motor controllers, sensors, vision systems, etc.
- **Specify versions**: Ensures everyone on the team uses the same library version
- **Download automatically**: Gradle downloads the libraries when you build
- **Maven/URL info**: Contains download URLs and Maven coordinates

## How to Add Vendor Libraries

### Method 1: Using WPILib VS Code Extension (Recommended)

1. Press `Ctrl+Shift+P` (or `Cmd+Shift+P` on Mac)
2. Type "WPILib: Manage Vendor Libraries"
3. Select "Install new libraries (online)" or "Install new libraries (offline)"
4. Choose the library from the list or paste a JSON URL

### Method 2: Manual Installation

1. Download the `.json` file from the vendor's website
2. Place it in this `vendordeps/` directory
3. Gradle will automatically detect and download the library on next build

## Common Vendor Libraries

### Motor Controllers & Hardware

- **REVLib** - REV Robotics (Spark Max, Spark Flex)
  - https://docs.revrobotics.com/sparkmax/software-resources/spark-max-api-information
- **Phoenix 6** - CTRE (Talon FX, CANcoder, Pigeon IMU)
  - https://pro.docs.ctr-electronics.com/en/stable/
- **NavX** - Kauai Labs (NavX Gyroscope)
  - https://pdocs.kauailabs.com/navx-mxp/software/roborio-libraries/

### Autonomous & Path Planning

- **PathPlannerLib** - Path planning for autonomous
  - https://pathplanner.dev/home.html
- **ChoreoLib** - Choreo trajectory generation
  - https://sleipnirgroup.github.io/Choreo/

### Vision Processing

- **PhotonLib** - PhotonVision camera processing
  - https://docs.photonvision.org/
- **Limelight** - Limelight vision camera
  - https://docs.limelightvision.io/

### Utilities

- **YAGSL** - Yet Another Generic Swerve Library
  - https://github.com/BroncBotz3481/YAGSL
- **AdvantageKit** - Logging and replay framework
  - https://github.com/Mechanical-Advantage/AdvantageKit

## Example Usage

After adding a vendordep file (e.g., `REVLib.json`), you can import and use it in your code:

```java
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class MySubsystem extends SubsystemBase {
    private CANSparkMax motor = new CANSparkMax(1, MotorType.kBrushless);

    // Your subsystem code here
}
```

## Troubleshooting

- **Library not found**: Make sure the `.json` file is in the `vendordeps/` directory
- **Build errors**: Try running "WPILib: Clean" then rebuild
- **Version conflicts**: Check that library versions are compatible with your WPILib year (2025)

## Resources

- [WPILib Third-Party Libraries](https://docs.wpilib.org/en/stable/docs/software/vscode-overview/3rd-party-libraries.html)
- [Vendor Library List](https://docs.wpilib.org/en/stable/docs/software/vscode-overview/3rd-party-libraries.html#libraries)
