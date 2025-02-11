// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.airlockConstants;

public class Airlock extends SubsystemBase {
    public TimeOfFlight frontTOF, backTOF;

  /** Creates a new airlock. */
  public Airlock() {
    frontTOF = new TimeOfFlight(airlockConstants.frontTOFCANID);
    backTOF = new TimeOfFlight(airlockConstants.backTOFCANID);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Front TOF range", frontTOF.getRange());
    SmartDashboard.putNumber("Back TOF range", backTOF.getRange());
  }

  /**@return front TOF sensors range*/
  public double getFrontRange(){
    return frontTOF.getRange();
  }
  /**@return back TOF sensors range*/
  public double getBackRange(){
    return backTOF.getRange();
  }
  /**@return true if the front TOF sensor's range is within defiend triggers*/
  public boolean isFrontInRange(){
    double range = getFrontRange();
    return range >= airlockConstants.backTOFTrigger[0] && range <= airlockConstants.backTOFTrigger[1];
  }
  /**@return true if the back TOF sensor's range is within defiend triggers*/
  public boolean isBackInRange(){
    double range = getBackRange();
    return range >= airlockConstants.backTOFTrigger[0] && range <= airlockConstants.backTOFTrigger[1];
  }
  /**@return true if its safe to move elevator*/
  public boolean checkSafety(){ 
    return isFrontInRange() && !isBackInRange();
  }
  /**@return true if a coral should be moved*/
  public boolean checkStep(){ 
    return !isFrontInRange() && isBackInRange();
  }
}
