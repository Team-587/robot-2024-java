// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

  public CANSparkMax m_RightClimbMotor = new CANSparkMax(Constants.OperatorConstants.kRightClimberMotorPort, MotorType.kBrushless);
  public CANSparkMax m_LeftClimbMotor = new CANSparkMax(Constants.OperatorConstants.kLeftClimberMotorPort, MotorType.kBrushless);


  boolean TentacleExtend = false;

  
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    m_RightClimbMotor.setIdleMode(IdleMode.kBrake);
    m_LeftClimbMotor.setIdleMode(IdleMode.kBrake);
    
    m_LeftClimbMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 100);
    m_LeftClimbMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 100);
    m_LeftClimbMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 100);
    m_RightClimbMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 100);
    m_RightClimbMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 100);
    m_RightClimbMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 100);
  }

  void allowClimberExtend() {
    TentacleExtend = !TentacleExtend;
  }

  void moveClimbers(double leftClimberValue, double rightClimberValue) {
    if(leftClimberValue < 0.1 && leftClimberValue > -0.1) {
      leftClimberValue = 0;
    }
    if(rightClimberValue < 0.1 && rightClimberValue > -0.1) {
      rightClimberValue = 0; 
    }

    if(TentacleExtend) {
      m_LeftClimbMotor.set(leftClimberValue);
      m_RightClimbMotor.set(rightClimberValue);
    } else {
      m_LeftClimbMotor.set(0);
      m_RightClimbMotor.set(0);
    }
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
