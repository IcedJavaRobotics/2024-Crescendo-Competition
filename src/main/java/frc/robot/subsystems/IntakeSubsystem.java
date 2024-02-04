// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  // Creates motors for intake and rollers
  public CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_SPARK_ID, MotorType.kBrushless);
  public VictorSPX rollerMotor = new VictorSPX(IntakeConstants.ROLLER_SPARK_ID);

  // Creates distance sensor and limit switches
  public DigitalInput distanceSensor = new DigitalInput(IntakeConstants.DISTANCE_SENSOR_ID);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeMotor.setInverted(IntakeConstants.INTAKE_MOTOR_INVERTED);
    rollerMotor.setInverted(IntakeConstants.ROLLER_MOTOR_INVERTED);
  }

  /**
   * Moves intake up
   */
  public void moveIntakeUp() {

    // if(intakeMotor.getEncoder().getPosition() < IntakeConstants.UPPER_ENCODER_LIMIT) {
    //   stopIntakeMotor();
    // } else {
    //   intakeMotor.set(IntakeConstants.INTAKE_SPEED);
    // }

     intakeMotor.set(IntakeConstants.INTAKE_SPEED);

  }

  /**
   * Moves intake down until lower limit switch is pressed
   */
  public void moveIntakeDown() {      
    
    // if(intakeMotor.getEncoder().getPosition() > IntakeConstants.LOWER_ENCODER_LIMIT) {
    //   stopIntakeMotor();
    // } else {
    //   intakeMotor.set(-IntakeConstants.INTAKE_SPEED);
    // }

    intakeMotor.set(-IntakeConstants.INTAKE_SPEED);

  }


  /**
   * Turns rollers in until piece is picked up
   * @return true if note is acquired, false if there is no note
   */
  public void turnRollerIn() {  // return type was boolean

    // rollerMotor.set(ControlMode.PercentOutput, IntakeConstants.ROLLER_SPEED);
    // if(havePiece()) { 
    //   stopRollerMotor();
    //   return true;
    // } else {
    //   return false;
    // }

    rollerMotor.set(ControlMode.PercentOutput, IntakeConstants.ROLLER_SPEED);

  }

  /**
   * Method for loading note into shooter
   */
  public void loadNote() {

    if (havePiece()){
      turnRollerOut();
    } else {
      stopRollerMotor();
    }

  }

  /**
   * Turns rollers out
   */
  public void turnRollerOut() {
    rollerMotor.set(ControlMode.PercentOutput, -IntakeConstants.ROLLER_SPEED);
  }
  
  /**
   * Turns rollers out at scoring speed
   */
  public void turnRollerScoring() {
    rollerMotor.set(ControlMode.PercentOutput, -IntakeConstants.SCORING_SPEED);
  }
  
  /**
   * Stops intake motor
   */
  public void stopIntakeMotor() {
    intakeMotor.set(0);
  }

  /**
   * Stops roller motor
   */
  public void stopRollerMotor() {
    rollerMotor.set(ControlMode.PercentOutput, 0);
  }

  /**
   * Determines if there is a piece in the robot
   * @return True if it detects a piece and false otherwise
   */
  public boolean havePiece() {
    return distanceSensor.get();
  }

/**
 * Resets the intake encoder to 0
 */
  public void zeroIntakeEncoder() {
    intakeMotor.getEncoder().setPosition(0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Encoder", intakeMotor.getEncoder().getPosition());
    SmartDashboard.putBoolean("Have Piece?", havePiece());
  }
}
