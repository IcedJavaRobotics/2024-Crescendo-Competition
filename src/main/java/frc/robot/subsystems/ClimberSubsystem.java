// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.Climber;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */

  private TalonSRX climberLeftMotor = new TalonSRX(Climber.LEFT_TALON);
  private TalonSRX climberRightMotor = new TalonSRX(Climber.RIGHT_TALON);

  DigitalInput climberLeftLimitSwitch = new DigitalInput(Climber.LEFT_LIMIT_SWITCH);
  DigitalInput climberRightLimitSwitch = new DigitalInput(Climber.RIGHT_LIMIT_SWITCH);

  public ClimberSubsystem() {

    climberRightMotor.setNeutralMode(null);;
    climberLeftMotor.setNeutralMode(null);

    climberLeftMotor.setInverted(false);
    climberRightMotor.setInverted(false);

  }

  /**
   * Variable speed command for right climber
   * @param speed Speed of the motor (double)
   */
  public void moveRightClimberMotor(double speed) {

    climberRightMotor.set(ControlMode.PercentOutput, speed);

  }

  /**
   * Variable speed command for left climber
   * @param speed Speed of the motor 
   */
  public void moveLeftClimberMotor(double speed) {

    climberLeftMotor.set(ControlMode.PercentOutput, speed);

  }

  /**
   * Moves both climbers up until encoder limits (in Constants).
   * Puts climber motor encoder values on the Smart Dashboard
   */
  public void moveClimberUp(){
    
    if(climberLeftMotor.getSelectedSensorPosition() <= Climber.UPPER_LEFT_LIMIT){

      moveLeftClimberMotor(Climber.SPEED);

    } else {

      moveLeftClimberMotor(0);

    }

    if(climberRightMotor.getSelectedSensorPosition() <= Climber.UPPER_RIGHT_LIMIT){

      moveRightClimberMotor(Climber.SPEED);

    } else {

      moveRightClimberMotor(0);

    }
    
    SmartDashboard.putNumber("Talon Left Climber", ( climberLeftMotor.getSelectedSensorPosition()));
    SmartDashboard.putNumber("Talon Right Climber", ( climberRightMotor.getSelectedSensorPosition()));

  }

  /**
   * Moves both climbers down until limit switch 
   */
  public void moveClimberDown(){
    
    if(climberLeftLimitSwitch.get() == false){

      moveLeftClimberMotor(-Climber.SPEED);

    } else {

      moveLeftClimberMotor(0);

    }

    if(climberRightLimitSwitch.get() == false){

      moveRightClimberMotor(-Climber.SPEED);

    } else {

      moveRightClimberMotor(0);

    }
    
  }
  
  /**
   * Stops climber movement 
   */
  public void climberStop() {

    climberLeftMotor.set(ControlMode.PercentOutput, 0);
    climberRightMotor.set(ControlMode.PercentOutput, 0);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
