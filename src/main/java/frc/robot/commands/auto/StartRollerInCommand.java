// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RollerSubsystem;

public class StartRollerInCommand extends Command {

  private final RollerSubsystem rollerSubsystem;

  /** Creates a new Flasher. */
  public StartRollerInCommand(RollerSubsystem r_Subsystem) {
    rollerSubsystem = r_Subsystem;
    addRequirements(rollerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rollerSubsystem.rollerMotor.set(ControlMode.PercentOutput, 0.85);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    rollerSubsystem.stopRollerMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(rollerSubsystem.havePiece()){
      return true;
    }

    return false;
  }
}
