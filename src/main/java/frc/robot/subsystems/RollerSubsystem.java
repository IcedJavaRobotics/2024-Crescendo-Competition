// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollerConstants;

public class RollerSubsystem extends SubsystemBase {
  /** Creates a new IntakeRollerSubsystem. */

  public VictorSPX rollerMotor = new VictorSPX(RollerConstants.ROLLER_SPARK_ID);
  public DigitalInput distanceSensor = new DigitalInput(RollerConstants.DISTANCE_SENSOR_ID);

  public RollerSubsystem() {
    rollerMotor.setInverted(RollerConstants.ROLLER_MOTOR_INVERTED);
  }

  /**
   * Turns rollers in until piece is picked up
   * @return true if note is acquired, false if there is no note
   */
  public void turnRollerIn() {  // return type was boolean
    rollerMotor.set(ControlMode.PercentOutput, RollerConstants.ROLLER_SPEED);
  }

  /**
   * Turns rollers out
   */
  public void turnRollerOut() {
    rollerMotor.set(ControlMode.PercentOutput, -RollerConstants.ROLLER_SPEED);
  }

  public void pickUpNote() {
    if(havePiece()) {
      stopRollerMotor();
    } else {
      turnRollerIn();
    }
  }

  /**
   * Determines if there is a piece in the robot
   * @return True if it detects a piece and false otherwise
   */
  public boolean havePiece() {
    return distanceSensor.get();
  }

  public void loadNote() {

    if (havePiece()){
      turnRollerOut();
    } else {
      stopRollerMotor();
    }

  }

  /**
   * Stops roller motor
   */
  public void stopRollerMotor() {
    rollerMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Have piece?", havePiece());
  }
}