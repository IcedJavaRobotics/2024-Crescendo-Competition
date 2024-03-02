// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;

public class PneumaticSubsystem extends SubsystemBase {
  /** Creates a new PneumaticSubsystem. */

  // Declaring solenoids and compressor
  DoubleSolenoid ampFlyswatter = new DoubleSolenoid(PneumaticsConstants.PNUEMATIC_HUB_ID, PneumaticsModuleType.REVPH, 2, 3);
  DoubleSolenoid climberRelease = new DoubleSolenoid(PneumaticsConstants.PNUEMATIC_HUB_ID, PneumaticsModuleType.REVPH, 0, 1);  
  // DoubleSolenoid ampFlyswatter = new DoubleSolenoid(PneumaticsModuleType.REVPH, 2, 3);
  // DoubleSolenoid climberRelease = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
  Compressor compressor = new Compressor(PneumaticsConstants.COMPRESSOR_ID, PneumaticsModuleType.REVPH);


  public PneumaticSubsystem() {
    
    // Setting default states, will occur when robot is enabled
    ampFlyswatter.set(DoubleSolenoid.Value.kReverse);
    climberRelease.set(DoubleSolenoid.Value.kForward);

  }

  public void releaseClimber() {

    climberRelease.set(DoubleSolenoid.Value.kForward);

  }

  public void lockClimber() {

    climberRelease.set(DoubleSolenoid.Value.kReverse);

  }

  public void ampScore() {

    ampFlyswatter.set(DoubleSolenoid.Value.kForward);

  }

  public void ampRetract() {

    ampFlyswatter.set(DoubleSolenoid.Value.kReverse);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run  
    SmartDashboard.putNumber("pressure", compressor.getPressure());
    SmartDashboard.putBoolean("climberLocked", !climberRelease.isRevSolenoidDisabled());
  }
}
