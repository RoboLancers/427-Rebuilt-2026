// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.parser.SwerveParser;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Distance;


public class LEDSubsystem extends SubsystemBase {


  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer;

  
  final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);

  LEDPattern redBlueGradient = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kRed, Color.kBlue);
  LEDPattern redBlueBreathe = redBlueGradient.breathe(Seconds.of(2));
  LEDPattern redBlueScroll = redBlueGradient.scrollAtRelativeSpeed(Percent.per(Seconds).of(25));


  LEDPattern oliveGreenGradient = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kGreen, Color.kDarkOliveGreen);
  LEDPattern oliveGreenBreathe = oliveGreenGradient.breathe(Seconds.of(2));
  LEDPattern oliveGreenScroll = oliveGreenGradient.scrollAtRelativeSpeed(Percent.per(Seconds).of(25));

  LEDPattern bluePinkYellowWhiteGradient = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kCornflowerBlue, Color.kPink, Color.kYellow, Color.kFloralWhite);
  LEDPattern bluePinkYellowWhiteBreathe = bluePinkYellowWhiteGradient.breathe(Seconds.of(2));
  LEDPattern bluePinkYellowWhiteScroll = bluePinkYellowWhiteGradient.scrollAtRelativeSpeed(Percent.per(Seconds).of(25));



  /* Blue Pink Yellow White
  Purple punk blue white */


  final Distance kLedSpacing = Meters.of(1 / 120);


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
    // Update the buffer with the rainbow animation
    //m_scrollingRainbow.applyTo(m_buffer);
    oliveGreenScroll.applyTo(m_buffer);
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
