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
    private Alert notSafeAlert = new Alert("coral is in airlock", AlertType.kWarning);
    public LaserCan frontLC, backLC;

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
    SmartDashboard.putNumber("Airlock/Front LC range", frontLC.getMeasurement().distance_mm);
    SmartDashboard.putNumber("Airlock/Back LC range", backLC.getMeasurement().distance_mm);
  }

  /**@return front TOF sensors range*/
  public double getFrontRange(){
    return frontLC.getMeasurement().distance_mm;
  }
  /**@return back TOF sensors range*/
  public double getBackRange(){
    return backLC.getMeasurement().distance_mm;
  }
  /**@return true if the front LC sensor's range is within defiend triggers*/
  public boolean isFrontInRange(){
    double range = getFrontRange();
    boolean test = range >= airlockConstants.backLCTrigger[0] && range <= airlockConstants.backLCTrigger[1];
    SmartDashboard.putBoolean("Airlock/front in range", test);
    return test;
  }
  /**@return true if the back LC sensor's range is within defiend triggers*/
  public boolean isBackInRange(){
    double range = getBackRange();
    boolean test = range >= airlockConstants.backLCTrigger[0] && range <= airlockConstants.backLCTrigger[1];
    SmartDashboard.putBoolean("Airlock/back in range", test);
    return test;
  }
  /**@return true if its safe to move elevator*/
  public boolean checkSafety(){ 
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
    return !isFrontInRange() && isBackInRange();
  }
}
