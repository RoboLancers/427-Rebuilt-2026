// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Led;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.Constants;
import swervelib.parser.SwerveParser;

import java.util.Map;

public class LEDSubsystem extends SubsystemBase {

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer;

  //Declares conditions which will be changed in commands
  public static boolean isIntaking = false;
  public static boolean isShooting = false; 



//Camera isConnected 
  public LEDSubsystem() {
    m_led = new AddressableLED(Constants.LedConstants.kPort);

    m_buffer = new AddressableLEDBuffer(Constants.LedConstants.kLength);
    m_led.setLength(m_buffer.getLength());

    m_led.setData(m_buffer);
    m_led.start();
    // Set the default command to turn the strip off, otherwise the last colors written by
    setDefaultCommand(runPattern(LEDPattern.solid(Color.kBlack)).withName("Off"));
  }

  @Override
  public void periodic() {
  LEDPattern decidedPattern = LEDPatterns.defaultPattern;
  
  
   //note, the higher up the condition, the lower the priority
   //if(isIntaking) decidedPattern = LEDPatterns.oliveGreenScroll;
   //if(isShooting) decidedPattern = LEDPatterns.bluePinkYellowWhiteScroll;

  decidedPattern.applyTo(m_buffer);
    // Set the LEDs
    m_led.setData(m_buffer);
  }
  /**
   * Creates a command that runs a pattern on the entire LED strip.
   *
   * @param pattern the LED pattern to run
   */
  public Command runPattern(LEDPattern pattern) {
    return run(() -> pattern.applyTo(m_buffer));
  }
}
