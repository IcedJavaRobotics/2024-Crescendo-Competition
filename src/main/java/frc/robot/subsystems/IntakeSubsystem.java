// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  public CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.SPARK_MAX_ID, MotorType.kBrushless);

  public DigitalInput intakeLimitSwitch = new DigitalInput(IntakeConstants.LIMIT_SWITCH_ID);

  public PIDController intakePidController;

  private double pidCalculations;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeMotor.setInverted(IntakeConstants.MOTOR_INVERTED);
  }

  /**
   * Moves intake up until limit switch is hit and resets encoder to 0
   */
  public void moveIntakeUp() {

    if(!intakeLimitSwitch.get()) {
      intakeMotor.set(0);
      zeroIntakeEncoder();
      return;
    }

    intakeMotor.set(IntakeConstants.SPEED);
  }

  /**
   * Moves intake down until lower limit is reached
   */
  public void moveIntakeDown() {      

    if(intakeMotor.getEncoder().getPosition() <= IntakeConstants.LOWER_ENCODER_LIMIT) {
      stopIntakeMotor();
      return;
    }

    intakeMotor.set(-IntakeConstants.SPEED);
  }

  public boolean flipperSpot() {

    if(intakeMotor.getEncoder().getPosition() <= IntakeConstants.FLIPPER_LIMIT) {
      stopIntakeMotor();
      return true;
    } else {
      intakeMotor.set(-IntakeConstants.SPEED);
      return false;
    }
  }
  
   
  /**
   * Stops intake motor
   */
  public void stopIntakeMotor() {
    intakeMotor.set(0);
  }


  /**
   * Zeros the intake encoder
   */
  public void zeroIntakeEncoder() {
    intakeMotor.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Encoder", intakeMotor.getEncoder().getPosition());
    SmartDashboard.putBoolean("Intake Limit Switch Pressed?", !intakeLimitSwitch.get());
  }
}
