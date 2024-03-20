// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeToAmpCommand extends Command {
  /** Creates a new AmpLoadCommand. */

  IntakeSubsystem intakeSubsystem;

  public IntakeToAmpCommand(IntakeSubsystem i_subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    intakeSubsystem = i_subsystem;
    addRequirements(i_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    intakeSubsystem.ampSpot();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stopIntakeMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if(intakeSubsystem.getIntakeEncoder() < -29 && intakeSubsystem.getIntakeEncoder() > -31) {
      return true;
    }

    return false;
  }
}
