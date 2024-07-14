// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterIntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShortShootCommand extends Command {
  private final ShooterSubsystem m_ShooterSubsystem;
  private final ShooterIntakeSubsystem m_ShooterIntakeSubsystem;
  /** Creates a new ShortShootCommand. */
  public ShortShootCommand(ShooterIntakeSubsystem shooterIntakeSubsystem, ShooterSubsystem shooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ShooterSubsystem = shooterSubsystem;
      addRequirements(m_ShooterSubsystem);
    m_ShooterIntakeSubsystem = shooterIntakeSubsystem;
      addRequirements(m_ShooterIntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ShooterSubsystem.setVelocity(Constants.OperatorConstants.ShortShootVelocity);
    m_ShooterIntakeSubsystem.setIntakeVelocity(Constants.OperatorConstants.intakeShootVelocity);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ShooterSubsystem.setVelocity(0);
    m_ShooterIntakeSubsystem.setIntakeVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
