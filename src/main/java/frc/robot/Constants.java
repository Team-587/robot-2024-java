// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCoDriverControllerPort = 1;
    public static final int kShooterPort = 1;
    public static final int kIntakePort = 2;
    public static final int kBumperIntakePort = 3;
    public static final int kLEDPort = 4;
    public static final int kIntakeSwitchPort = 5;
    public static final int kRightClimberMotorPort = 6;
    public static final int kLeftClimberMotorPort = 7;
    public static final int kElavatorMotorPort = 8;
    public static final int kElbowAMotorPort = 9;
    public static final int kElbowBMotorPort = 10;

    public static final int kLEDSideLength = 26;
    public static final int kLEDBackLength = 36;
    public static final int kLEDTotalLength = kLEDSideLength + kLEDSideLength + kLEDBackLength;

    public static final double ShortShootVelocity = 4800.00;
    public static final double FeedShootVelocity = 6000.00;
    public static final double LongShootVelocity = 5200.00;
    public static final double AmpShootVelocity = 3500.00;
    public static final double TrapShootVelocity = 900.00;

    public static final double intakeVelocity = 0.45;
    public static final double groundIntakeVelocity = 0.55;
    public static final double intakeShootVelocity = 1.00;
    
  }
}
