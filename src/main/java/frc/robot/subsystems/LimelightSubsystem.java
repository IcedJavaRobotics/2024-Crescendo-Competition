// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;

import static frc.robot.Constants.LimelightConstants;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;

public class LimelightSubsystem extends SubsystemBase {

  // private Spark blinkin = new Spark(0); //Creates a blinkin as if it were a
  // spark.
  // Creates a new LimelightSubsystem.
  public LimelightSubsystem() {

    HttpCamera lemonLight = new HttpCamera("CoprocessorCamera", "http://limelight.local:5801/");
    CameraServer.getVideo(lemonLight); 
    Shuffleboard.getTab("LiveWindow").add(lemonLight);
    
    //CameraServer.getInstance().addCamera(httpCamera); Shuffleboard.getTab("Tab") .add(httpCamera);
    
  }

  //the id of the apriltag
  public double getTid() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0);
  }

  //the x away from the center

  public double getTx() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  }

  //the y away from the center
  public double getTy() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  }

  //the size of the apriltag
  public double getTa() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
  }
  /**
   * 
   * @param mode 1.0 for on, 0.0 for off
   */
  public void setFlasher(double mode){
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setDouble(mode);
  }

  /**
   * whether or not the limelight sees a apriltag
   * 
   * @return true if it sees an apriltag, false if the obvious happens
   */
  public Boolean tagDetected() {

    if (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1) {
      return true;
    }
    return false;
  }

  /**
   * command for calculating limelight distance
   * 
   * @return horizontal floor distance from robot to apriltag
   */
  public double getDistance() {

    double targetOffsetAngle_Vertical = getTy();
    double goalHeightInches = LimelightConstants.APRILTAG_HEIGHT;
    double angleToGoalRadians = (LimelightConstants.LIMELIGHT_ANGLE + targetOffsetAngle_Vertical) * (3.14159 / 180);

    if(getTid() == LimelightConstants.APRILTAG_SPEAKER_ID){
      goalHeightInches = LimelightConstants.APRILTAG_SPEAKER_HEIGHT;
    }
    else if(getTid() == LimelightConstants.APRILTAG_AMP_ID){
      goalHeightInches = LimelightConstants.APRILTAG_AMP_HEIGHT;
    }
    // calculate distance
    if (getTy() >= 0) {
      return (goalHeightInches - LimelightConstants.LIMELIGHT_HEIGHT) / Math.tan(angleToGoalRadians);
    } else {
      return (LimelightConstants.LIMELIGHT_HEIGHT) / Math.tan(angleToGoalRadians);
    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    
  }
}
