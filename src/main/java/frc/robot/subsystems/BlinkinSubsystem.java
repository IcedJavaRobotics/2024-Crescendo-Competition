// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.BlinkinConstants;

public class BlinkinSubsystem extends SubsystemBase {
  /** Creates a new BlinkinSubsystem. */
  private Spark blinkin; 
  public BlinkinSubsystem() {

    blinkin = new Spark(BlinkinConstants.BLINKIN);

  }
  

  /**
   * <p> Turns the Blinkin Red
   **/
  public void turnBlinkinRed(){

    blinkin.set(BlinkinConstants.RED);

  }

  /**
   * <p> Turns the Blinkin Blue 
   */
  public void turnBlinkinBlue(){

    blinkin.set(BlinkinConstants.BLUE);

  }

  /**
   * <p> Turns the Blinkin Green 
   */
  public void turnBlinkinGreen(){

    blinkin.set(BlinkinConstants.GREEN);

  }

  /**
   * <p> Turns the Blinkin Rainbow 
   */
  public void turnBlinkinRainbow(){

  blinkin.set(BlinkinConstants.RAINBOW);

  }

  /**
   * <p> Turns the Blinkin Ocean
   */
  public void turnBlinkinOcean(){

    blinkin.set(BlinkinConstants.OCEAN);

  }

  /**
   * <p> Turns the Blinkin Forest 
   */
  public void turnBlinkinForest(){

  blinkin.set(BlinkinConstants.FOREST);     

  }

  public void turnBlinkinAllianceColor(){
      Optional<Alliance> ally = DriverStation.getAlliance();
      if(ally.isPresent()) {

          if(ally.get() == Alliance.Red){
            turnBlinkinRed();
          }
          if(ally.get() == Alliance.Blue){
            turnBlinkinBlue();
          }

      }
      else{
        turnBlinkinOcean();
      }
  }

  /**
   * <p> Turns Blinkin red if the robot alliance is red and turns Blinkin blue if the robot alliance is blue. If there is no robot alliance color then Blinkin turns forest. 
   */
  public void autoBlinkin(){
    Optional<Alliance> color = DriverStation.getAlliance();
   if (color.isPresent()){
      if (color.get() == Alliance.Red){
          turnBlinkinRed();

      }

      if (color.get() == Alliance.Blue){
          turnBlinkinBlue();
      }
   }
    else {
          turnBlinkinForest();
     }

    }
    


    public void intakeColor(boolean havePiece){
        if(havePiece){
          turnBlinkinForest();
        } else{
          turnBlinkinOff();
        }
    }
  
    public void turnBlinkinOff() {
      blinkin.set(BlinkinConstants.BLACK);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
