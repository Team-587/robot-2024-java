// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
   CANSparkMax m_ElavatorMotor = new CANSparkMax(Constants.OperatorConstants.kElavatorMotorPort, MotorType.kBrushless);
   CANSparkMax m_ElbowAMotor = new CANSparkMax(Constants.OperatorConstants.kElbowAMotorPort, MotorType.kBrushless);
   CANSparkMax m_ElbowBMotor = new CANSparkMax(Constants.OperatorConstants.kElbowBMotorPort, MotorType.kBrushless);
  }

  void ArmPosition(double angle, double height) {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  static final double PickUpAngle = 0.2;
  static final double PickUpLength = 0.6;
  static final double HoldAngle = 0.2;
  static final double HoldLength = 0.6;
  static final double AmpAngle = 83.0;
  static final double AmpLength = 7.9;
  static final double ShortShootAngle = 3.0;
  static final double ShortShootLength = 0.6;
  static final double LongShootAngle = 19.0;
  static final double LongShootLength = 0.6;
  static final double TrapShootingAngle = 60.0;

  
}
