// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterIntakeSubsystem extends SubsystemBase {

  MotorController m_intakeMotor = new CANSparkFlex(Constants.OperatorConstants.kIntakePort, CANSparkLowLevel.MotorType.kBrushless);

  /** Creates a new ShooterIntakeSubsysten. */
  public ShooterIntakeSubsystem() {}

  public void setVelocity(double velocity) {
    m_intakeMotor.set(velocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
