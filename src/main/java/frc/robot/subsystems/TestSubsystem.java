// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestSubsystem extends SubsystemBase {
  /** Creates a new TestSubsystem. */

  private final CANSparkMax motor = new CANSparkMax(1, MotorType.kBrushless);

  public TestSubsystem() {
    motor.setInverted(false);
  }

  public void moveMotor() {
    motor.set(0.1);
  }

  public void stopMotor() {
    motor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
