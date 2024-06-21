// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterIntakeSubsystem extends SubsystemBase {

  static final double ShortShootVelocity = 4800.00;
  static final double FeedShootVelocity = 6000.00;
  static final double LongShootVelocity = 5200.00;
  static final double AmpShootVelocity = 3500.00;
  static final double TrapShootVelocity = 900.00;
  static final double StopShootVelocity = 0.0;

  static final double intakeVelocity = 0.45;
  static final double groundIntakeVelocity = 0.55;
  static final double intakeShootVelocity = 1.00;

  DigitalInput intakeSwitch = new DigitalInput(Constants.OperatorConstants.kIntakeSwitchPort);

  MotorController m_intakeMotor = new CANSparkFlex(Constants.OperatorConstants.kIntakePort, CANSparkLowLevel.MotorType.kBrushless);

  /** Creates a new ShooterIntakeSubsysten. */
  public ShooterIntakeSubsystem() {}

  public void setIntakeVelocity(double velocity) {
    m_intakeMotor.set(velocity);
  }

  public boolean getIntakeSensorState() { return !intakeSwitch.get(); };

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
