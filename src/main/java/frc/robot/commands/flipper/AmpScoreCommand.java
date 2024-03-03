// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.flipper;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PneumaticsConstants;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.RollerSubsystem;

public class AmpScoreCommand extends Command {
  /** Creates a new AmpScoreCommand. */

  private final PneumaticSubsystem pneumaticSubsystem;

  double scoreTime;

  public AmpScoreCommand(PneumaticSubsystem p_subsystem) {
    pneumaticSubsystem = p_subsystem;
    addRequirements(pneumaticSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pneumaticSubsystem.ampScore();
    scoreTime = Timer.getMatchTime();
    pneumaticSubsystem.initSpeedDisabler(Timer.getMatchTime());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if ((scoreTime - Timer.getMatchTime())> PneumaticsConstants.AMP_HOLD_TIME) {
      pneumaticSubsystem.ampRetract();
    } else {
      pneumaticSubsystem.ampWaiting();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
