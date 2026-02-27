// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Led;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import java.util.Map;

public class LEDPatterns {


  public static LEDPattern defaultPattern = LEDPattern.solid(Color.kBlack);

    public static LEDPattern red = LEDPattern.solid(Color.kRed);
    public static LEDPattern orange = LEDPattern.solid(Color.kOrange);
    public static LEDPattern yellow = LEDPattern.solid(Color.kYellow);
    public static LEDPattern green = LEDPattern.solid(Color.kGreen);
    public static LEDPattern blue = LEDPattern.solid(Color.kBlue);
    public static LEDPattern pink = LEDPattern.solid(Color.kPink);

    public static LEDPattern redBreathe = red.breathe(Seconds.of(2));
    public static LEDPattern blueBreathe = blue.breathe(Seconds.of(2));
    
    public static LEDPattern seizure =
        LEDPattern.steps(
            Map.of(
                0, Color.kRed,
                0.125, Color.kOrange,
                0.25, Color.kYellow,
                0.375, Color.kGreen,
                0.5, Color.kBlue,
                0.625, Color.kPink,
                0.75, Color.kRed,
                0.875, Color.kOrange));

  public static LEDPattern bluePinkYellowWhiteGradient =
      LEDPattern.gradient(
          LEDPattern.GradientType.kContinuous,
          Color.kCornflowerBlue,
          Color.kPink,
          Color.kYellow,
          Color.kFloralWhite);
  public static LEDPattern bluePinkYellowWhiteBreathe =
      bluePinkYellowWhiteGradient.breathe(Seconds.of(2));
  public static LEDPattern bluePinkYellowWhiteScroll =
      bluePinkYellowWhiteGradient.scrollAtRelativeSpeed(Percent.per(Seconds).of(15));

  public static LEDPattern purplePinkBlueWhiteGradient =
      LEDPattern.gradient(
          LEDPattern.GradientType.kContinuous,
          Color.kMediumPurple,
          Color.kPink,
          Color.kCornflowerBlue,
          Color.kFloralWhite);
  public static LEDPattern purplePinkBlueWhiteBreathe =
      purplePinkBlueWhiteGradient.breathe(Seconds.of(2));
  public static LEDPattern purplePinkBlueWhiteScroll =
      purplePinkBlueWhiteGradient.scrollAtRelativeSpeed(Percent.per(Seconds).of(15));

  public static LEDPattern yellowRedSteps =
      LEDPattern.steps(
          Map.of(0, Color.kRed, 0.25, Color.kYellow, 0.5, Color.kRed, 0.75, Color.kYellow));
  public static LEDPattern yellowRedScroll =
      yellowRedSteps.scrollAtRelativeSpeed(Percent.per(Seconds).of(25));

  // scaledInputs.getX(), Constants.

}
