// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */

  private TalonSRX climberLeftMotor = new TalonSRX(ClimberConstants.LEFT_TALON);
  private TalonSRX climberRightMotor = new TalonSRX(ClimberConstants.RIGHT_TALON);

  public DigitalInput leftLimitSwitch = new DigitalInput(2);
  public DigitalInput rightLimitSwitch = new DigitalInput(3);

  public ClimberSubsystem() {

    climberRightMotor.setNeutralMode(NeutralMode.Brake);
    climberLeftMotor.setNeutralMode(NeutralMode.Brake);

    climberLeftMotor.setInverted(ClimberConstants.LEFT_MOTOR_INVERTED);
    climberRightMotor.setInverted(ClimberConstants.RIGHT_MOTOR_INVERTED);

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
    
    if(climberLeftMotor.getSelectedSensorPosition() < ClimberConstants.UPPER_LEFT_LIMIT){

      moveLeftClimberMotor(ClimberConstants.SPEED);

    } else {

      leftClimberStop();

    }

    if(climberRightMotor.getSelectedSensorPosition() < ClimberConstants.UPPER_RIGHT_LIMIT){

      moveRightClimberMotor(ClimberConstants.SPEED);

    } else {

      rightClimberStop();

    }

  }

  /**
   * Moves both climbers down until limit switch 
   */
  public void moveClimberDown() {
    
    if(!leftLimitSwitch.get() == true) {

      leftClimberStop();
      climberLeftMotor.setSelectedSensorPosition(0);

    } else {

      moveLeftClimberMotor(-ClimberConstants.SPEED);

    }

    if(!rightLimitSwitch.get() == true) {

      rightClimberStop();
      climberRightMotor.setSelectedSensorPosition(0);

    } else {

      moveRightClimberMotor(-ClimberConstants.SPEED);

    }
    
    
  }

  public boolean leftSwitchPressed() {
    return !leftLimitSwitch.get();
  }

  public boolean rightSwitchPressed() {
    return !rightLimitSwitch.get();
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
    SmartDashboard.putBoolean("LeftClimbSwitch", !leftLimitSwitch.get());
    SmartDashboard.putBoolean("RightClimbSwitch", !rightLimitSwitch.get());
  }
}
