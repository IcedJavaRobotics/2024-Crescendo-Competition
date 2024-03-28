// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RollerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AmpShootCommand extends Command {
  /** Creates a new AmpShootCommand. */
  private final ShooterSubsystem shooterSubsystem;
  private final RollerSubsystem rollerSubsystem;
  private final PneumaticSubsystem pneumaticSubsystem;

  public AmpShootCommand(ShooterSubsystem sSubsystem, RollerSubsystem rSubsystem, PneumaticSubsystem pSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    shooterSubsystem = sSubsystem;
    rollerSubsystem = rSubsystem;
    pneumaticSubsystem = pSubsystem;
    addRequirements(sSubsystem, rSubsystem, pSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pneumaticSubsystem.ampScore();
    shooterSubsystem.setSpeed(ShooterConstants.AMP_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooterSubsystem.getSpeed() < (ShooterConstants.AMP_SPEED)){
      shooterSubsystem.shooterAtMaxSpeed = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    //if the shooter is warmed up to less than 0.6 times the desired speed, dont fire anyway
    if(shooterSubsystem.getSpeed() < (ShooterConstants.AMP_SPEED)){
      shooterSubsystem.setSpeed(0); //stop the motor in that case
    } else {   //if its more then that, fire anyway

      rollerSubsystem.rollerMotor.set(ControlMode.PercentOutput, -RollerConstants.MEDIUM_SPEED);
      shooterSubsystem.initSpeedDisabler(System.currentTimeMillis()); //waits a second before setting the speed to 0.
      rollerSubsystem.initSpeedDisabler(System.currentTimeMillis());
      pneumaticSubsystem.initSpeedDisabler(System.currentTimeMillis());

    }

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
