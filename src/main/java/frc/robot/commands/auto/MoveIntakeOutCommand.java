// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class MoveIntakeOutCommand extends Command {
  /** Creates a new MoveIntakeOutCommand. */

  IntakeSubsystem intakeSubsystem;

  public MoveIntakeOutCommand(IntakeSubsystem i_subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    intakeSubsystem = i_subsystem;
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.moveIntakeToSetPosition(-80);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stopIntakeMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(intakeSubsystem.getIntakeEncoder() < -79 && intakeSubsystem.getIntakeEncoder() > -81) {
      return true;
    }

    return false;
  }
}
