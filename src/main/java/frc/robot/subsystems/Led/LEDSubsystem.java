// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Led;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.Map;
import edu.wpi.first.math.geometry.Translation2d;





public class LEDSubsystem extends SubsystemBase {

  private static final int kPort = 9;
  private static final int kLength = 60;

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer;

    final Distance kLedSpacing = Meters.of(1 / 120);

  //Rainbow pattern (Kinda broken)
   LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);
  //LEDPattern brighten = base.atBrightness(Percent.of(170));
  //LEDPattern breathe = brighten.breathe(Seconds.of(3));
  //LEDPattern m_rainbow = breathe.scrollAtAbsoluteSpeed(Centimeters.per(Seconds).of(12.5), kLedSpacing);


  //Solid Red pattern
  final LEDPattern m_red = LEDPattern.solid(Color.kRed);

  //Gradient pattern
  final LEDPattern m_gradientBase = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kGreen, Color.kBlack, Color.kGainsboro);
  LEDPattern m_gradientScroll =  m_gradientBase.scrollAtRelativeSpeed(Percent.per(Seconds).of(25));
  LEDPattern m_gradient = m_gradientScroll.atBrightness(Percent.of(100));


 // final LEDPattern xSpeed = LEDPattern.progressMaskLayer(() -> scaledInputs.getX() / Constants.DriveConstants.MAX_SPEED);


  //Scrolling rainbow pattern (also broken)
   LEDPattern m_scrollingRainbow =
    m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);

  public LEDSubsystem() {
    m_led = new AddressableLED(kPort);

    m_buffer = new AddressableLEDBuffer(kLength);
    m_led.setLength(m_buffer.getLength());

    m_led.setData(m_buffer);
    m_led.start();
    
    // Set the default command to turn the strip off, otherwise the last colors written by
    // the last command to run will continue to be displayed.
    // Note: Other default patterns could be used instead!
    // setDefaultCommand(runPattern(LEDPattern.solid(Color.kBlack)).withName("Off"));
  }

  @Override
  public void periodic() {

    m_gradient.applyTo(m_buffer); //Change the value here to change the pattern!!!!!!!! XD
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
