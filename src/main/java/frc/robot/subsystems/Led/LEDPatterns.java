// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Led;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Map;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.parser.SwerveParser;

public class LEDPatterns {

  public static LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);
  public static LEDPattern defaultPattern = LEDPattern.solid(Color.kBlack);

  public static LEDPattern oliveGreenGradient = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kGreen, Color.kDarkOliveGreen);
  public static LEDPattern oliveGreenBreathe = oliveGreenGradient.breathe(Seconds.of(2));
  public static LEDPattern oliveGreenScroll = oliveGreenGradient.scrollAtRelativeSpeed(Percent.per(Seconds).of(25));

  public static LEDPattern bluePinkYellowWhiteGradient = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kCornflowerBlue, Color.kPink, Color.kYellow, Color.kFloralWhite);
  public static LEDPattern bluePinkYellowWhiteBreathe = bluePinkYellowWhiteGradient.breathe(Seconds.of(2));
  public static LEDPattern bluePinkYellowWhiteScroll = bluePinkYellowWhiteGradient.scrollAtRelativeSpeed(Percent.per(Seconds).of(15));

  public static LEDPattern purplePinkBlueWhiteGradient = LEDPattern.gradient(LEDPattern.GradientType.kContinuous,Color.kMediumPurple, Color.kPink, Color.kCornflowerBlue, Color.kFloralWhite);
  public static LEDPattern purplePinkBlueWhiteBreathe = purplePinkBlueWhiteGradient.breathe(Seconds.of(2));
  public static LEDPattern purplePinkBlueWhiteScroll = purplePinkBlueWhiteGradient.scrollAtRelativeSpeed(Percent.per(Seconds).of(15));

  public static LEDPattern yellowRedSteps = LEDPattern.steps(Map.of(0, Color.kRed, 0.25, Color.kYellow,0.5, Color.kRed, 0.75, Color.kYellow));
  public static LEDPattern yellowRedScroll = yellowRedSteps.scrollAtRelativeSpeed(Percent.per(Seconds).of(25));





//scaledInputs.getX(), Constants.







}
