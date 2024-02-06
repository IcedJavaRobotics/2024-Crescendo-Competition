// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  public CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_SPARK_ID, MotorType.kBrushless);
  //public CANSparkMax rollerMotor = new CANSparkMax(IntakeConstants.ROLLER_MOTOR_ID, MotorType.kBrushless);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {

    intakeMotor.setInverted(IntakeConstants.INTAKE_MOTOR_INVERTED);
    //rollerMotor.setInverted(IntakeConstants.ROLLER_MOTOR_INVERTED);
  }

  /**
   * Moves intake up until upper limit is reached
   */
  public void moveIntakeUp() {

    if(intakeMotor.getEncoder().getPosition() < IntakeConstants.UPPER_ENCODER_LIMIT) {
      intakeMotor.set(IntakeConstants.INTAKE_SPEED);
    } else {
      stopIntakeMotor();
    }

    // intakeMotor.set(IntakeConstants.INTAKE_SPEED);
  }

  /**
   * Moves intake down until lower limit is reached
   */
  public void moveIntakeDown() {      
    
    if(intakeMotor.getEncoder().getPosition() > IntakeConstants.LOWER_ENCODER_LIMIT) {
      intakeMotor.set(-IntakeConstants.INTAKE_SPEED);
    } else {
      stopIntakeMotor();
    }

    // intakeMotor.set(-IntakeConstants.INTAKE_SPEED);
  }

  // /**
  //  * Turns rollers in at speed .2
  //  */
  // public void turnRollerIn() {
  //   rollerMotor.set(.2);
  // }

  // /**
  //  * Turns rollers out at speed .2
  //  */
  // public void turnRollerOut() {
  //   rollerMotor.set(-.2);
  // }
  
  
  /**
   * Stops intake motor
   */
  public void stopIntakeMotor() {
    intakeMotor.set(0);
  }

  // /**
  //  * Stops roller motor
  //  */
  // public void stopRollerMotor() {
  //   rollerMotor.set(0);
  // }

  public void zeroIntakeEncoder() {
    intakeMotor.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Encoder", intakeMotor.getEncoder().getPosition());
  }
}
