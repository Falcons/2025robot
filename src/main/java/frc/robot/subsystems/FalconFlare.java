// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.falconFlareConstants;

public class FalconFlare extends SubsystemBase {
  DigitalOutput D1 = new DigitalOutput(falconFlareConstants.dio1);
  DigitalOutput D2 = new DigitalOutput(falconFlareConstants.dio2);
  DigitalOutput D3 = new DigitalOutput(falconFlareConstants.dio3);
  Map<String,Boolean[]> colours = new HashMap<String,Boolean[]>();
  Optional<DriverStation.Alliance> Alliance = DriverStation.getAlliance();
  Timer timer = new Timer();
  /** Creates a new FalconFlare. */
  public FalconFlare() {
    colours.put("white", new Boolean[]{false,true,false});
    colours.put("green", new Boolean[]{false,true,true});
    colours.put("purple", new Boolean[]{true,false,false});
    colours.put("yellow", new Boolean[]{true,false,true});
    colours.put("blue", new Boolean[]{true,true,false});
    colours.put("red", new Boolean[]{true,true,true});
    setLights("blue");
    timer.start();
  }

  @Override
  public void periodic() {
    if(DriverStation.isDisabled() && timer.get() > 60)reset();
  }

  public void setLights(boolean in1, boolean in2, boolean in3){
    D1.set(in1);
    D2.set(in2);
    D3.set(in3);
  }
  /**
   * @param colour white, green, purple, yellow, blue, red
   */
  public void setLights(String colour){
    Boolean[] data = colours.get(colour);
    D1.set(data[0]);
    D2.set(data[1]);
    D3.set(data[2]);
  }
  public void reset() {
    boolean isRed = Alliance.get() == DriverStation.Alliance.Red;
    setLights(false, false, isRed);
  }
  public void dance(){}
}

