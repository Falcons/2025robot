// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.airlockConstants;

public class Airlock extends SubsystemBase {
    public Alert notSafeAlert = new Alert("coral is in airlock", AlertType.kWarning);
    public Alert stepAlert = new Alert("steping coral", AlertType.kInfo);
    public LaserCan  frontLC, backLC;
    public boolean keepStep;

  /** Creates a new airlock. */
  public Airlock() {
    frontLC = new LaserCan(airlockConstants.frontLCCANID);
    backLC = new LaserCan(airlockConstants.backLCCANID);
    try{
      frontLC.setRangingMode(RangingMode.SHORT);
      backLC.setRangingMode(RangingMode.SHORT);
    }catch(ConfigurationFailedException e){
      System.err.print(e);
    }
    checkSafety();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("airlock/Front LC range", getFrontRange());
    SmartDashboard.putNumber("airlock/Back LC range", getBackRange());
  }

  /**@return front TOF sensors range*/
  public double getFrontRange(){
    try {
      return frontLC.getMeasurement().distance_mm;
    } catch (Exception e) {
      System.err.println(e);
      return 9999;
    }
  }
  /**@return back TOF sensors range*/
  public double getBackRange(){
    try {
      return backLC.getMeasurement().distance_mm;
    } catch (Exception e) {
      System.err.println(e);
      return 9999;
    }
    
  }
  /**@return true if the front LC sensor's range is within defiend triggers*/
  public boolean isFrontInRange(){
    double range = getFrontRange();
    boolean test = range >= airlockConstants.backLCTrigger[0] && range <= airlockConstants.backLCTrigger[1];
    SmartDashboard.putBoolean("airlock/front in range", test);
    return test;
  }
  /**@return true if the back LC sensor's range is within defiend triggers*/
  public boolean isBackInRange(){
    double range = getBackRange();
    boolean test = range >= airlockConstants.backLCTrigger[0] && range <= airlockConstants.backLCTrigger[1];
    SmartDashboard.putBoolean("airlock/back in range", test);
    return test;
  }
  /**@return true if its safe to move elevator*/
  public boolean checkSafety(){ //lights
    if (!isFrontInRange() && !isBackInRange()) {notSafeAlert.set(false); return true;}
    if (isFrontInRange() && !isBackInRange()) {
      notSafeAlert.set(false);
      return true;
    }else {
      notSafeAlert.set(true);
      return false;
    }
  }
  /**@return true if a coral should be moved*/
  public boolean checkStep(){ 
    if(keepStep){ 
      if (isFrontInRange() && !isBackInRange()) {
        keepStep = false;
        stepAlert.set(false);
        return true;
      }else{
      stepAlert.set(true);
      return true;
      }
    }
    if(isBackInRange()){
      keepStep = true;
      stepAlert.set(true);
      return true;
    }else {
      keepStep = false;
      stepAlert.set(false);
      return false;
    }
  }
}
