// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Led;


import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.Comparator;
import java.util.TreeSet;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class Led extends SubsystemBase {
  /** Creates a new Led. */
  //Add vairables to constant.java instead of here
  private static final int kPort = 9;
  private static final int kLength = 60;
  
  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer;
  
  public Led() {
    m_led = new AddressableLED(kPort);
    m_buffer = new AddressableLEDBuffer(kLength);
    m_led.setLength(kLength);
    m_led.start();

    //Set the defualt command to turn the strip off, 
    //otherwise the last colors wirtten by the last command will continue to be displayed.
   //ote, other default partterns could be used instead
    setDefaultCommand(runPattern(LEDPattern.solid(Color.kBlack)).withName("Off"));
  }

  @Override
  public void periodic() {
    // Periodically send the latest LED color data to the LED strip for it to display
    m_led.setData(m_buffer);
  }

  public Command runPattern(LEDPattern pattern) {
    return run(() -> pattern.applyTo(m_buffer));
  }
}
