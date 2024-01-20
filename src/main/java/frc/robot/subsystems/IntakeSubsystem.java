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
  public CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
  public CANSparkMax rollerMotor = new CANSparkMax(IntakeConstants.ROLLER_MOTOR_ID, MotorType.kBrushless);

  // Creates distance sensor and limit switches
  public Ultrasonic distanceSenor = new Ultrasonic(0, 1);
  public DigitalInput upperLimitSwitch = new DigitalInput(2);
  public DigitalInput lowerLimitSwitch = new DigitalInput(3);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeMotor.setInverted(IntakeConstants.INTAKE_MOTOR_INVERTED);
    rollerMotor.setInverted(IntakeConstants.ROLLER_MOTOR_INVERTED);
  }

  /**
   * Moves intake up at speed .2
   */
  public void moveIntakeUp() {
    if(upperLimitSwitch.get() == true) {
      stopIntakeMotor();
    } else {
      intakeMotor.set(.2);
    }
  }

  /**
   * Moves intake down at speed .2 until lower limit switch is pressed
   */
  public void moveIntakeDown() {
    if(lowerLimitSwitch.get() == true) {
      stopIntakeMotor();
    } else {
      intakeMotor.set(-.2);
    }
  }

  /**
   * Turns rollers in at speed .2 until piece is picked up
   */
  public void turnRollerIn() {
    if(distanceSenor.getRangeMM() <= 5) { 
      stopRollerMotor();
    } else {
      rollerMotor.set(.2);
    }
  }

  /**
   * Turns rollers out at speed .2
   */
  public void turnRollerOut() {
    rollerMotor.set(-.2);
  }
  
  /**
   * Turns scoring rollers out at speed .1
   */
  public void turnRollerScoring() {
    rollerMotor.set(-.1);
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
    if(distanceSenor.getRangeMM() <= 5) {
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
