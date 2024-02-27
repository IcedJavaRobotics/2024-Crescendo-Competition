// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RollerSubsystem;


public class PickupNoteCommand extends Command {

  private final IntakeSubsystem intakeSubsystem;
  private final RollerSubsystem rollerSubsystem;

  /** Creates a new PickupNoteCommand. */
  public PickupNoteCommand(IntakeSubsystem subsystem, RollerSubsystem subsystem2) {
    intakeSubsystem = subsystem;
    rollerSubsystem = subsystem2;
    addRequirements(intakeSubsystem);
    addRequirements(rollerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    intakeSubsystem.moveIntakeOut();
    rollerSubsystem.pickUpNote();
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    rollerSubsystem.stopRollerMotor();
    intakeSubsystem.moveIntakeIn();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
