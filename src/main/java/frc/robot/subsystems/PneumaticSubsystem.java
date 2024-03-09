// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;

public class PneumaticSubsystem extends SubsystemBase {
  /** Creates a new PneumaticSubsystem. */

  // Declaring solenoids and compressor
  Solenoid ampFlyswatter = new Solenoid(PneumaticsConstants.PNUEMATIC_HUB_ID, PneumaticsModuleType.REVPH, 2);
  DoubleSolenoid climberRelease = new DoubleSolenoid(PneumaticsConstants.PNUEMATIC_HUB_ID, PneumaticsModuleType.REVPH, 0, 1);  
  Compressor compressor = new Compressor(PneumaticsConstants.COMPRESSOR_ID, PneumaticsModuleType.REVPH);

  private double timeScore;      //Tracks when controller is pressed
  private boolean ampWaitingToCooldown = false;

  public PneumaticSubsystem() {
    
    // Setting default states, will occur when robot is enabled
    ampFlyswatter.set(false);
    climberRelease.set(DoubleSolenoid.Value.kForward);

  }

  public void releaseClimber() {

    climberRelease.set(DoubleSolenoid.Value.kForward);

  }

  public void lockClimber() {

    climberRelease.set(DoubleSolenoid.Value.kReverse);

  }

  public void ampScore() {

    ampFlyswatter.set(true);

  }

  public void ampRetract() {

    ampFlyswatter.set(false);

  }

  public void initSpeedDisabler(double initMatchTime) {
    this.timeScore = initMatchTime;
  }

  public void ampWaiting() {
    this.ampWaitingToCooldown = true;
  }

  public void cooldownFlyswatter(){
    if(ampWaitingToCooldown) {
      //If it has been COOLDOWN_TIME amount of time since fired set speed to 0
      if((timeScore-Timer.getMatchTime()) < PneumaticsConstants.AMP_HOLD_TIME) {
        ampFlyswatter.set(false);
        this.ampWaitingToCooldown = false;
      }
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run  
    SmartDashboard.putNumber("pressure", compressor.getPressure());
    SmartDashboard.putBoolean("climberLocked", !climberRelease.isRevSolenoidDisabled());
  }
}
