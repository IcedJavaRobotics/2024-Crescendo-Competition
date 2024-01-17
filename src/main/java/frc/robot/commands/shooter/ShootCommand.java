// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.Timer;
import static frc.robot.Constants.ShooterConstants;

public class ShootCommand extends Command {
  
  private final ShooterSubsystem shooterSubsystem;
  private int SecondsWarmedUp = 0;
  /** Creates a new Shoe. */
  public ShootCommand(ShooterSubsystem shSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterSubsystem = shSubsystem;
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.setSpeed(ShooterConstants.DESIRED_SPEAKER_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    //if the shooter is warmed up to less than 0.6 times the desired speed, dont fire anyway
    if(shooterSubsystem.getSpeed() < (ShooterConstants.DESIRED_SPEAKER_SPEED * 0.6)){
      shooterSubsystem.setSpeed(0); //stop the motor in that case
    } else{   //if its more then that, fire anyway
      //load note command goes here!! //TODO
      shooterSubsystem.initSpeedDisabler(Timer.getMatchTime());

      
    }

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
