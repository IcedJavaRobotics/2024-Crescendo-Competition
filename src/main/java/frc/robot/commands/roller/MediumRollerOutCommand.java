// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.roller;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RollerConstants;
import frc.robot.subsystems.RollerSubsystem;

public class MediumRollerOutCommand extends Command {
  /** Creates a new SlowRollerInCommand. */

  private final RollerSubsystem rollerSubsystem;

  public MediumRollerOutCommand(RollerSubsystem rSubsystem) {
    rollerSubsystem = rSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rollerSubsystem.rollerMotor.set(ControlMode.PercentOutput, -RollerConstants.MEDIUM_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    rollerSubsystem.rollerMotor.set(ControlMode.PercentOutput, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
