// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class StopShootingCommand extends Command {
  /** Creates a new StopShootingCommand. */

  ShooterSubsystem shooterSubsystem;
  RollerSubsystem rollerSubsystem;
  public StopShootingCommand(ShooterSubsystem s_subsystem, RollerSubsystem r_subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    shooterSubsystem = s_subsystem;
    rollerSubsystem = r_subsystem;

    addRequirements(shooterSubsystem);
    addRequirements(rollerSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.setSpeed(0);
    rollerSubsystem.rollerMotor.set(ControlMode.PercentOutput, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
