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
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

   m_ElevatorPIDController = m_ElevatorMotor.getPIDController();
   m_ElbowAnglePIDController = m_ElbowAMotor.getPIDController();

   m_ElevatorAbsoluteEncoder = m_ElevatorMotor.getAbsoluteEncoder();
   m_ElbowAbsoluteEncoder = m_ElbowAMotor.getAbsoluteEncoder();

   m_ElevatorHeight = 0.6;
   m_ElbowAngle = 0.2;
   m_CurElevatorHeight = 0.0;
   m_CurElbowAngle = 0.0;

   m_ElevatorAbsoluteEncoder.setPositionConversionFactor(10.2);
   m_ElevatorAbsoluteEncoder.setVelocityConversionFactor(10.2 / 60.0);

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
   m_ElevatorMotor.setSoftLimit(SoftLimitDirection.kForward, (float) 8.3);
   m_ElevatorMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) 0.4);
   m_ElevatorMotor.setIdleMode(IdleMode.kBrake);


   m_ElbowAbsoluteEncoder.setPositionConversionFactor(360);
   m_ElbowAbsoluteEncoder.setVelocityConversionFactor((360)/60.0);

   m_ElbowAnglePIDController.setFeedbackDevice(m_ElbowAbsoluteEncoder);
   m_ElbowAnglePIDController.setPositionPIDWrappingEnabled(true);
   m_ElbowAnglePIDController.setPositionPIDWrappingMinInput(0);
   m_ElbowAnglePIDController.setPositionPIDWrappingMaxInput(360);
   m_ElbowAnglePIDController.setP(elbowP);
   m_ElbowAnglePIDController.setI(elbowI);
   m_ElbowAnglePIDController.setD(elbowD);
   m_ElbowAnglePIDController.setFF(0);
   m_ElbowAnglePIDController.setOutputRange(-1.0, 1.0);


   m_ElbowAMotor.setOpenLoopRampRate(0.5);
   m_ElbowAMotor.setSmartCurrentLimit(60);
   m_ElbowAMotor.setSoftLimit(SoftLimitDirection.kForward, (float) ElbowAngleMax);
   m_ElbowAMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) ElbowAngleMin);
   m_ElbowAMotor.setIdleMode(IdleMode.kBrake);
   m_ElbowAMotor.setInverted(true);

   m_ElbowBMotor.follow(m_ElbowAMotor);
   //m_ElbowBMotor.setInverted(false);
   m_ElbowBMotor.setOpenLoopRampRate(0.5);
   m_ElbowBMotor.setSmartCurrentLimit(60);
   m_ElbowBMotor.setSoftLimit(SoftLimitDirection.kForward, (float) ElbowAngleMax);
   m_ElbowBMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) ElbowAngleMin);

   elbowPID.setP(elbowP);
   elbowPID.setI(elbowI);
   elbowPID.setD(elbowD);
   SmartDashboard.putData("elbowPID", elbowPID);
   elevatorPID.setP(elevatorP);
   elevatorPID.setI(elevatorI);
   elevatorPID.setD(elevatorD);
   SmartDashboard.putData("elevatorPID", elevatorPID);
   m_ElevatorMotor.burnFlash();
  }

  void ArmPosition(double angle, double height) 
  {
    if (height > ElevatorMax) height = ElevatorMax;
    if (height < ElevatorMin) height = ElevatorMin;
    m_ElevatorHeight = height;

    if (angle > ElbowAngleMax) angle = ElbowAngleMax;
    if (angle < ElbowAngleMin) angle = ElbowAngleMin;
    m_ElbowAngle = angle;

    System.out.println(" Repositioning Arm");
    System.out.println(" Change Arm Angle");
  }

  void GetArmPos(double angle, double height) {
    angle = m_CurElbowAngle;
    height = m_CurElevatorHeight;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    XboxController m_codriverController = new XboxController(Constants.OperatorConstants.kCoDriverControllerPort);
    if(m_codriverController.getBackButton()){
      
      double coLeftY = m_codriverController.getLeftY();
      if(coLeftY > -0.1 && coLeftY < 0.1){
        coLeftY = 0;
      }
      double coRightY = m_codriverController.getRightY();
      if(coRightY > -0.1 && coRightY < 0.1) {
        coRightY = 0;
      }

      m_ElbowAngle = m_ElbowAngle + coLeftY*4.0;
      if (m_ElbowAngle > ElbowAngleMax) m_ElbowAngle = ElbowAngleMax;
      if (m_ElbowAngle < ElbowAngleMin) m_ElbowAngle = ElbowAngleMin;

      if (coRightY < 0) coRightY = 0;
      m_ElevatorHeight = m_ElevatorHeight + coRightY*0.5;
      if (m_ElevatorHeight > m_ElevatorHeight) m_ElevatorHeight = ElevatorMax;
      if(m_ElevatorHeight < m_ElevatorHeight) m_ElevatorHeight = ElevatorMin;
    }

    if(elevatorP != elevatorPID.getP() || elevatorI != elevatorPID.getI() || elevatorD != elevatorPID.getD()) {
      elevatorP = elevatorPID.getP();
      elevatorI = elevatorPID.getI();
      elevatorD = elevatorPID.getD();

      elevatorPID.setP(elevatorP);
      elevatorPID.setI(elevatorI);
      elevatorPID.setD(elevatorD);
    }

    if(elbowP != elbowPID.getP() || elbowI != elbowPID.getI() || elbowD != elbowPID.getD()) {
      elbowP = elbowPID.getP();
      elbowI = elbowPID.getI();
      elbowD = elbowPID.getD();

      m_ElbowAnglePIDController.setP(elbowP);
      m_ElbowAnglePIDController.setI(elbowI);
      m_ElbowAnglePIDController.setD(elbowD);
    }

    double ElevatorAbsoluteEncoderValue = m_ElevatorAbsoluteEncoder.getPosition();
    double ElbowAbsoluteEncoderValue = m_ElbowAbsoluteEncoder.getPosition();

    m_CurElevatorHeight = ElevatorAbsoluteEncoderValue;
    m_CurElbowAngle = ElbowAbsoluteEncoderValue;


    SmartDashboard.putNumber("Cur ElbowAngle", ElbowAbsoluteEncoderValue);
    SmartDashboard.putNumber("Set Elbow Angle", m_ElbowAngle);
    SmartDashboard.putNumber("Cur Elev Height", ElevatorAbsoluteEncoderValue);
    SmartDashboard.putNumber("Set Elev Height", m_ElevatorHeight);


    boolean CurElevatorLong = ElevatorAbsoluteEncoderValue >= ElevatorMaxSafe;
    boolean CurElbowHigh = ElbowAbsoluteEncoderValue >= ElbowAngleMaxSafe;
    boolean DesElevatorLong = m_ElevatorHeight >= ElevatorMaxSafe;
    boolean DesElbowHigh = m_ElbowAngle >= ElbowAngleMaxSafe;


    m_ElbowAnglePIDController.setReference(m_ElbowAngle, ControlType.kPosition);
    m_ElevatorPIDController.setReference(m_ElevatorHeight, ControlType.kPosition);
  } 

  

  
}
