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
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.Constants;

//Code first, think later o7

public class ArmSubsystem extends SubsystemBase {

  private final CANSparkMax m_ElevatorMotor;
  private final CANSparkMax m_ElbowAMotor;
  private final CANSparkMax m_ElbowBMotor;
  
  SparkPIDController m_ElevatorPIDController;
  SparkPIDController m_ElbowAnglePIDController;

  SparkAbsoluteEncoder m_ElevatorAbsoluteEncoder;
  SparkAbsoluteEncoder m_ElbowAbsoluteEncoder;

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

  private static final double ElevatorMin = 0.4;
  private static final double ElevatorMax = 8.3;
  private static final double ElevatorMaxSafe = 5.0;
  private static final double ElbowAngleMin = 0.0;
  private static final double ElbowAngleMax = 91.0;
  private static final double ElbowAngleMaxSafe = 14.0;

  double elbowP = 0.035;
  double elbowI = 0.0;
  double elbowD = 0.0;

  PIDController elbowPID = new PIDController(elbowP, elbowI, elbowD);

  double elevatorP = 0.2;
  double elevatorI = 0.0;
  double elevatorD = 0.0;

  PIDController elevatorPID = new PIDController(elevatorP, elevatorI, elevatorD);

  double m_ElevatorHeight;
  double m_ElbowAngle;
  double m_CurElevatorHeight;
  double m_CurElbowAngle;
  

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
   m_ElevatorMotor = new CANSparkMax(Constants.OperatorConstants.kElavatorMotorPort, MotorType.kBrushless);
   m_ElbowAMotor = new CANSparkMax(Constants.OperatorConstants.kElbowAMotorPort, MotorType.kBrushless);
   m_ElbowBMotor = new CANSparkMax(Constants.OperatorConstants.kElbowBMotorPort, MotorType.kBrushless);

   m_ElevatorPIDController = m_ElavatorMotor.getPIDController();
   m_ElbowAnglePIDController = m_ElbowAMotor.getPIDController();

   m_ElevatorAbsoluteEncoder = m_ElavatorMotor.getAbsoluteEncoder();
   m_ElbowAbsoluteEncoder = m_ElbowAMotor.getAbsoluteEncoder();

   m_ElevatorHeight = 0.6;
   m_ElbowAngle = 0.2;
   m_CurElevatorHeight = 0.0;
   m_CurElbowAngle = 0.0;

   m_ElevatorAbsoluteEncoder.setPositionConversionFactor(Constants.OperatorConstants.kElevatorPIDPosConversionFact);
   m_ElevatorAbsoluteEncoder.setVelocityConversionFactor(Constants.OperatorConstants.kElevatorPIDPosConversionFact / 60.0);

   m_ElevatorPIDController.setFeedbackDevice(m_ElevatorAbsoluteEncoder);
   m_ElevatorPIDController.setPositionPIDWrappingEnabled(false);
   m_ElevatorPIDController.setPositionPIDWrappingMinInput(0.4);
   m_ElevatorPIDController.setPositionPIDWrappingMaxInput(11);
   m_ElevatorPIDController.setP(elevatorP);
   m_ElevatorPIDController.setI(elevatorI);
   m_ElevatorPIDController.setD(elevatorD);
   m_ElevatorPIDController.setFF(0);
   m_ElevatorPIDController.setOutputRange(-1.0, 1.0);
   m_ElevatorMotor.setInverted(true);
   //m_ElevatorMotor.SetOpenLoopRampRate(0.5);
   m_ElevatorMotor.setSmartCurrentLimit(60);
   m_ElevatorMotor.setSoftLimit(SoftLimitDirection.kForward, 8.3);
   m_ElevatorMotor.setSoftLimit(SoftLimitDirection.kReverse, 0.4);
   m_ElevatorMotor.setIdleMode(IdleMode.kBrake);
  }

  void ArmPosition(double angle, double height) {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  

  
}
