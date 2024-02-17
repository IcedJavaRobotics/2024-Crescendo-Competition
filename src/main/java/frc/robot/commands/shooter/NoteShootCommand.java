// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.Timer;
import static frc.robot.Constants.ShooterConstants;

public class NoteShootCommand extends Command {
  
  private final ShooterSubsystem shooterSubsystem;
  private final RollerSubsystem rollersubsystem; 
  
  /** Creates a new Shoe. */
  public NoteShootCommand(ShooterSubsystem subsystem, RollerSubsystem subsystem2) {
    // Use addRequirements() here to declare subsystem dependencies.);
    shooterSubsystem = subsystem;
    rollersubsystem = subsystem2;
    addRequirements(shooterSubsystem);
    addRequirements(rollersubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.setSpeed(ShooterConstants.SPEAKER_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    //if the shooter is warmed up to less than 0.6 times the desired speed, dont fire anyway
    if(shooterSubsystem.getSpeed() < (ShooterConstants.SPEAKER_SPEED * 0.9)){
      shooterSubsystem.setSpeed(0); //stop the motor in that case
    } else {   //if its more then that, fire anyway
      
      rollersubsystem.loadNote();
      shooterSubsystem.initSpeedDisabler(Timer.getMatchTime()); //waits a second before setting the speed to 0.

    }

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
