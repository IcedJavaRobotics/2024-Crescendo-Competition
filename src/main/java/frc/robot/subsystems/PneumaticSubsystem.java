// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;

public class PneumaticSubsystem extends SubsystemBase {
  /** Creates a new PneumaticSubsystem. */
  DoubleSolenoid ampPlaceholderName = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 0);
  DoubleSolenoid leftClimberRelease = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 0);
  DoubleSolenoid rightClimberRelease =  new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 0);
  Compressor compressor = new Compressor(PneumaticsConstants.COMPRESSOR_ID, PneumaticsModuleType.REVPH);

  public PneumaticSubsystem() {

    ampPlaceholderName.set(DoubleSolenoid.Value.kReverse);
    leftClimberRelease.set(DoubleSolenoid.Value.kReverse);
    rightClimberRelease.set(DoubleSolenoid.Value.kReverse);

  }

  public void releaseClimber() {

    leftClimberRelease.set(DoubleSolenoid.Value.kReverse);
    rightClimberRelease.set(DoubleSolenoid.Value.kReverse);

  }

  public void lockClimber() {

    leftClimberRelease.set(DoubleSolenoid.Value.kForward);
    rightClimberRelease.set(DoubleSolenoid.Value.kForward);

  }

  public void ampScore() {

    ampPlaceholderName.set(DoubleSolenoid.Value.kForward);

  }

  public void ampRetract() {

    ampPlaceholderName.set(DoubleSolenoid.Value.kReverse);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
