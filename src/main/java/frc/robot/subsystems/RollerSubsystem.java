// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollerConstants;
import frc.robot.Constants.ShooterConstants;

public class RollerSubsystem extends SubsystemBase {
  /** Creates a new IntakeRollerSubsystem. */

  public VictorSPX rollerMotor = new VictorSPX(RollerConstants.VICTOR_ID);
  public DigitalInput distanceSensor = new DigitalInput(RollerConstants.DISTANCE_SENSOR_ID);

  private double timeShot;      //Tracks when controller trigger is let go
  private boolean rollerWaitingToCooldown = false;

  public static boolean loading = false;

  public RollerSubsystem() {
    rollerMotor.setInverted(RollerConstants.MOTOR_INVERTED);
    rollerMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void slowRollIn() {
    rollerMotor.set(ControlMode.PercentOutput, 0.2);
  }

  /**
   * Turns rollers in until piece is picked up
   * @return true if note is acquired, false if there is no note
   */
  public void turnRollerIn() {  // return type was boolean

    if(havePiece()) {
      stopRollerMotor();
    } else {
      rollerMotor.set(ControlMode.PercentOutput, RollerConstants.INTAKE_SPEED);
    }
  }

  public void turnRollerInNoLimit() {
    rollerMotor.set(ControlMode.PercentOutput, RollerConstants.INTAKE_SPEED);
  }

  /**
   * Turns rollers out
   */
  public void turnRollerOut(double Speed) {
    rollerMotor.set(ControlMode.PercentOutput, Speed);
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
    return !distanceSensor.get();
  }

  // public boolean loadShooter() {

  //   if (havePiece()){
  //     turnRollerOut(-RollerConstants.SHOOTER_SPEED);
  //     return true;
  //   } else {
  //     stopRollerMotor();
  //     return false;
  //   }

  // }

  public void  loadFlipper() {

    turnRollerOut(-RollerConstants.FLIPPER_SPEED);

  }

  public void ejectNote(){
    turnRollerOut(-RollerConstants.EJECT_SPEED);
  }

  /**
   * Stops roller motor
   */
  public void stopRollerMotor() {
    rollerMotor.set(ControlMode.PercentOutput, 0);
  }

  public void setSpeed(double speed) {
    rollerMotor.set(ControlMode.PercentOutput, speed);
  }

  public void initSpeedDisabler(double initMatchTime) {
    this.timeShot = initMatchTime;
    this.rollerWaitingToCooldown = true;
  }

  public void cooldownRoller(){
    if(rollerWaitingToCooldown) {
      //If it has been COOLDOWN_TIME amount of time since fired set speed to 0
      if((timeShot-Timer.getMatchTime()) > ShooterConstants.COOLDOWN_TIME) {
        stopRollerMotor();
        this.rollerWaitingToCooldown = false;
      }
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Have piece?", havePiece());
  }
}
