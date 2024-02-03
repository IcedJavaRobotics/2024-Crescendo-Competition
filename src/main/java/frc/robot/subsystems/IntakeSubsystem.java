// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  // Creates motors for intake and rollers
  public CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_SPARK_ID, MotorType.kBrushless);
  public CANSparkMax rollerMotor = new CANSparkMax(IntakeConstants.ROLLER_SPARK_ID, MotorType.kBrushless);

  // Creates distance sensor and limit switches
  public Ultrasonic distanceSenor = new Ultrasonic(IntakeConstants.ULTRASONIC_PING_PORT, IntakeConstants.ULTRASONIC_ECHO_PORT);
  public DigitalInput upperLimitSwitch = new DigitalInput(IntakeConstants.UPPER_LIMIT_SWITCH);
  public DigitalInput lowerLimitSwitch = new DigitalInput(IntakeConstants.LOWER_LIMIT_SWITCH);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeMotor.setInverted(IntakeConstants.INTAKE_MOTOR_INVERTED);
    rollerMotor.setInverted(IntakeConstants.ROLLER_MOTOR_INVERTED);
  }

  /**
   * Moves intake up
   */
  public void moveIntakeUp() {

    if(upperLimitSwitch.get() == true) {
      stopIntakeMotor();
    } else {
      intakeMotor.set(IntakeConstants.INTAKE_SPEED);
    }

  }

  /**
   * Moves intake down until lower limit switch is pressed
   */
  public void moveIntakeDown() {      
    
    if(lowerLimitSwitch.get() == true) {
      stopIntakeMotor();
    } else {
      intakeMotor.set(-IntakeConstants.INTAKE_SPEED);
    }

  }

  /**
   * Turns rollers in until piece is picked up
   * @return true if note is acquired, false if there is no note
   */
  public boolean turnRollerIn() {

    rollerMotor.set(IntakeConstants.ROLLER_SPEED);
    if(havePiece()) { 
      stopRollerMotor();
      return true;
    } else {
      return false;
    }

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
    rollerMotor.set(-IntakeConstants.ROLLER_SPEED);
  }
  
  /**
   * Turns rollers out at scoring speed
   */
  public void turnRollerScoring() {
    rollerMotor.set(-IntakeConstants.SCORING_SPEED);
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
    rollerMotor.set(0);
  }

  public boolean havePiece() {
    if(distanceSenor.getRangeMM() <= IntakeConstants.ULTRASONIC_RANGE) {
      return true;
    }
    return false;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("IntakeUpperSwitch", upperLimitSwitch.get());
    SmartDashboard.putBoolean("IntakeLowerSwitch", lowerLimitSwitch.get());
    SmartDashboard.putBoolean("IntakeLowerSwitch", havePiece());
  }
}
