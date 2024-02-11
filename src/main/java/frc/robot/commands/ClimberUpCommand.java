// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberUpCommand extends Command {
  /** Creates a new ClimberUpCommand. */

  private final ClimberSubsystem climberSubsystem;

  public ClimberUpCommand( ClimberSubsystem subsystem ) {
    // Use addRequirements() here to declare subsystem dependencies.
    climberSubsystem = subsystem;
    addRequirements(climberSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climberSubsystem.moveClimberUp();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climberSubsystem.climberStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
