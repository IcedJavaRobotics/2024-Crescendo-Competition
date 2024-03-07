// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class StartShooterCommand extends Command {
  /** Creates a new StartShooterCommand. */

  ShooterSubsystem shooterSubsystem;
  // double time = 0;
  
  public StartShooterCommand(ShooterSubsystem s_subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    shooterSubsystem = s_subsystem;
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // time = Timer.getMatchTime();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.setSpeed(0.85);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if(Math.abs(time - Timer.getMatchTime()) > 2) {
    //   return true;
    // }
    // return false;
    return false;
  }
}
