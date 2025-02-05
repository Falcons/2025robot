// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.airlockConstants;

public class airlock extends SubsystemBase {
    public TimeOfFlight frontTOF, backTOF;

  /** Creates a new airlock. */
  public airlock() {
    frontTOF = new TimeOfFlight(airlockConstants.frontTOFCANID);
    backTOF = new TimeOfFlight(airlockConstants.backTOFCANID);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Front TOF range", frontTOF.getRange());
    SmartDashboard.putNumber("Back TOF range", backTOF.getRange());
  }

  public double getFrontRange(){
    return frontTOF.getRange();
  }
  public double getBackRange(){
    return backTOF.getRange();
  }

  public boolean isFrontInRange(){
    double range = frontTOF.getRange();
    return range >= airlockConstants.backTOFTriggerMin && range <= airlockConstants.backTOFTriggerMax;
  }
  public boolean isBackInRange(){
    double range = backTOF.getRange();
    return range >= airlockConstants.backTOFTriggerMin && range <= airlockConstants.backTOFTriggerMax;
  }
  public boolean checkSafety(){
    return isFrontInRange() && !isBackInRange();
  }
}
