// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.security.PublicKey;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.falconFlareConstants;

public class FalconFlare extends SubsystemBase {
  DigitalOutput D1 = new DigitalOutput(falconFlareConstants.dio1);
  DigitalOutput D2 = new DigitalOutput(falconFlareConstants.dio2);
  DigitalOutput D3 = new DigitalOutput(falconFlareConstants.dio3);
  /** Creates a new FalconFlare. */
  public FalconFlare() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setLights(boolean in1, boolean in2, boolean in3){
    D1.set(in1);
    D2.set(in2);
    D3.set(in3);
  }
}
