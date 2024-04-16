// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  MotorController m_shooterMotor = new CANSparkMax(Constants.OperatorConstants.kShooterPort, CANSparkLowLevel.MotorType.kBrushless);

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {}

  public void setVelocity(double velocity) {
      m_shooterMotor.set(velocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
