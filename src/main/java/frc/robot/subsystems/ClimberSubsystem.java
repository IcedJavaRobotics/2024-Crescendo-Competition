// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

import static frc.robot.Constants.ClimberConstants;;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */

  private TalonSRX climberLeftMotor = new TalonSRX(ClimberConstants.LEFT_TALON);
  private TalonSRX climberRightMotor = new TalonSRX(ClimberConstants.RIGHT_TALON);

  DigitalInput climberLeftLimitSwitch = new DigitalInput(ClimberConstants.LEFT_LIMIT_SWITCH);
  DigitalInput climberRightLimitSwitch = new DigitalInput(ClimberConstants.RIGHT_LIMIT_SWITCH);

  public ClimberSubsystem() {

    climberRightMotor.setNeutralMode(NeutralMode.Brake);;
    climberLeftMotor.setNeutralMode(NeutralMode.Brake);

    climberLeftMotor.setInverted(false);
    climberRightMotor.setInverted(false);

  }

  /**
   * Variable speed command for right climber
   * @param speed Speed of the motor (double)
   */
  public void moveRightClimberMotor() {

    climberRightMotor.set(ControlMode.PercentOutput, ClimberConstants.SPEED);

  }

  /**
   * Variable speed command for left climber
   * @param speed Speed of the motor 
   */
  public void moveLeftClimberMotor() {

    climberLeftMotor.set(ControlMode.PercentOutput, ClimberConstants.SPEED);

  }

  /**
   * Moves both climbers up until encoder limits (in Constants).
   * Puts climber motor encoder values on the Smart Dashboard
   */
  public void moveClimberUp(){
    
    if(climberLeftMotor.getSelectedSensorPosition() < ClimberConstants.UPPER_LEFT_LIMIT){

      moveLeftClimberMotor();

    } else {

      leftClimberStop();

    }

    if(climberRightMotor.getSelectedSensorPosition() < ClimberConstants.UPPER_RIGHT_LIMIT){

      moveRightClimberMotor();

    } else {

      rightClimberStop();

    }

  }

  /**
   * Moves both climbers down until limit switch 
   */
  public void moveClimberDown(){
    
    if(climberLeftLimitSwitch.get() == false){

      moveLeftClimberMotor();

    } else {

      leftClimberStop();

    }

    if(climberRightLimitSwitch.get() == false){

      moveRightClimberMotor();

    } else {

      rightClimberStop();
    }
    
  }
  
  public void leftClimberStop() {

    climberLeftMotor.set(ControlMode.PercentOutput, 0);
    
  }

  public void rightClimberStop() {

    climberRightMotor.set(ControlMode.PercentOutput, 0);
    
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
    SmartDashboard.putNumber("Talon Left Climber Encoder", ( climberLeftMotor.getSelectedSensorPosition()));
    SmartDashboard.putNumber("Talon Right Climber Encoder", ( climberRightMotor.getSelectedSensorPosition()));
  }
}
