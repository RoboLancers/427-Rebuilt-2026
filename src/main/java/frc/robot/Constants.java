package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class DriveConstants { // Robot Physical Constants & Speed
    public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
    public static final Matter CHASSIS =
        new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag}

    public static final double MAX_SPEED =
        Units.feetToMeters(15); // this used to be like 14 or smth
    public static final double MAX_ANGULAR_SPEED = Units.feetToMeters(15);
    // Joystick deadband
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

  
    public static class LedConstants {
     public static final int kPort = 9;
     public static final int kLength = 60;
    }
   
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final String drivebase = "drivebase";
    // this used to be 6 if turning speed is stupid make it six
  }

}
