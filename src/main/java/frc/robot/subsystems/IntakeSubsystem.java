// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  public CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
  public CANSparkMax rollerMotor = new CANSparkMax(IntakeConstants.ROLLER_MOTOR_ID, MotorType.kBrushless);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {

    intakeMotor.setInverted(false);
    rollerMotor.setInverted(false);
  }

  public void moveIntakeUp (){
    intakeMotor.set(.2);
  }
  public void moveIntakeDown (){
    intakeMotor.set(-.2);
  }
  public void turnRollersIn(){
    rollerMotor.set(.2);
  }
  public void turnRollersOut(){
    rollerMotor.set(-.2);
  }
  public void stopIntakeMotor (){
    intakeMotor.set(0);
  }
  public void stopRollerMotor(){
    rollerMotor.set(0);
  }

  public void turnScoringRollersOut () {
    rollerMotor.set(-.1);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
