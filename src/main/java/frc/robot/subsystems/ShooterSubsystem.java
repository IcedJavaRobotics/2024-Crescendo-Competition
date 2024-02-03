// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import static frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;

    private double timeShot;      //Tracks when controller trigger is let go
    private boolean shooterWaitingToCooldown = false;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    leftMotor = new CANSparkMax(ShooterConstants.kLeftMotorPort,CANSparkLowLevel.MotorType.kBrushless);
    rightMotor = new CANSparkMax(ShooterConstants.kRightMotorPort,CANSparkLowLevel.MotorType.kBrushless);
    rightMotor.setInverted(true);
    leftMotor.setInverted(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    cooldownShooter();
  }

  public void setSpeed(double speed){
    leftMotor.set(speed);
    rightMotor.set(speed);
  }
  public double getSpeed(){
    //test if motors are same speed, if not, print message
    if(leftMotor.getEncoder().getVelocity() != rightMotor.getEncoder().getVelocity()){
      System.out.println("Motors are at an uneven speed!");
    }
    return leftMotor.getEncoder().getVelocity();

  }
  
  public void initSpeedDisabler(double initMatchTime){
    this.timeShot = initMatchTime;
    this.shooterWaitingToCooldown = true;
  }
  public void cooldownShooter(){
    if(shooterWaitingToCooldown){
      //If it has been COOLDOWN_TIME amount of time since fired set speed to 0
      if((timeShot-Timer.getMatchTime()) > ShooterConstants.COOLDOWN_TIME){
        setSpeed(0);
        this.shooterWaitingToCooldown = false;
      }
    }
  }
}
