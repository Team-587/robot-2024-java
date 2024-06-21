// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterIntakeSubsystem;
import frc.robot.Constants;

public class ShooterCommand extends Command {
  private final ShooterIntakeSubsystem m_shooterIntakeSubsystem;
  /** Creates a new ShooterCommand. */
  public ShooterCommand(ShooterIntakeSubsystem shooterIntakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooterIntakeSubsystem = shooterIntakeSubsystem;
      addRequirements(m_shooterIntakeSubsystem);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterIntakeSubsystem.setIntakeVelocity(Constants.OperatorConstants.intakeShootVelocity);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterIntakeSubsystem.setIntakeVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
