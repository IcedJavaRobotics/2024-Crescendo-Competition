// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;

public class ClimberMiddleCommand extends Command {

  private final LimelightSubsystem limelightSubsystem;
  /** Creates a new Flasher. */
  public ClimberMiddleCommand(LimelightSubsystem lSubsystem) {
    this.limelightSubsystem = lSubsystem;
    addRequirements(limelightSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //System.out.println("damn thats crazy");
    limelightSubsystem.setFlasher(3.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    limelightSubsystem.setFlasher(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
