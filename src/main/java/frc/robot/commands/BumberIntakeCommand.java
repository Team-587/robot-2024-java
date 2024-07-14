// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.constant.Constable;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.BumperIntakeSubsystem;
import frc.robot.subsystems.ShooterIntakeSubsystem;

public class BumberIntakeCommand extends Command {
  private final BumperIntakeSubsystem m_BumperIntakeSubsystem;
  private final ShooterIntakeSubsystem m_ShooterIntakeSubsystem;
  /** Creates a new BumberIntakeCommand. */
  public BumberIntakeCommand(BumperIntakeSubsystem bumperIntakeSubsystem, ShooterIntakeSubsystem shooterIntakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_BumperIntakeSubsystem = bumperIntakeSubsystem;
      addRequirements(m_BumperIntakeSubsystem);
    m_ShooterIntakeSubsystem = shooterIntakeSubsystem;
    addRequirements(m_ShooterIntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_BumperIntakeSubsystem.setVelocity(Constants.OperatorConstants.groundIntakeVelocity);
    m_ShooterIntakeSubsystem.setIntakeVelocity(Constants.OperatorConstants.intakeVelocity);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_BumperIntakeSubsystem.setVelocity(0);
    m_ShooterIntakeSubsystem.setIntakeVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
