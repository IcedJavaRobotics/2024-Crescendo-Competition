// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.flipper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RollerSubsystem;

public class LoadFlipperCommand extends Command {
  /** Creates a new LoadFlipperCommand. */
  
  RollerSubsystem rollerSubsystem;
  IntakeSubsystem intakeSubsystem;

  public LoadFlipperCommand(RollerSubsystem subsystem, IntakeSubsystem intakeSubsystem) {
    rollerSubsystem = subsystem;
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(rollerSubsystem);
    addRequirements(this.intakeSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.flipperSpot();
    if (intakeSubsystem.flipperSpot()) {
      rollerSubsystem.loadFlipper();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    rollerSubsystem.stopRollerMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
