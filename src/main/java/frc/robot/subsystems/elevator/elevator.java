// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.airlock;

public class elevator extends SubsystemBase {
    private final SparkMax leftMoter, rightMoter;
    private SparkMaxConfig leftConfig, rightConfig = new SparkMaxConfig();
    private TimeOfFlight TOF = new TimeOfFlight(ElevatorConstants.TOFTopCANID);
    private Alert slowModeAlert = new Alert("Elevator Slow Mode Active", Alert.AlertType.kInfo);
    private double maxSpeed = 1;
    /** Creates a new elevator. */
  public elevator(airlock airlock) {
    this.rightMoter = new SparkMax(ElevatorConstants.liftMoter1CANID, MotorType.kBrushless);
    rightConfig.idleMode(IdleMode.kBrake);
    rightMoter.configure(rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    this.leftMoter = new SparkMax(ElevatorConstants.liftMoter2CANID, MotorType.kBrushless);
    leftConfig.idleMode(IdleMode.kBrake);
    leftConfig.follow(rightMoter);
    leftConfig.inverted(true);
    leftMoter.configure(leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("left encoder", leftMoter.getEncoder().getPosition());
    SmartDashboard.putNumber("right encoder", rightMoter.getEncoder().getPosition());
    SmartDashboard.putNumber("TOF range", TOF.getRange());
  }

  public void set(double speed){ //TODO: add safety checks
    rightMoter.set(speed * maxSpeed);
  }
  public void stop(){
    rightMoter.stopMotor();
  }
  public void setSlowMode(boolean toggle){
    if (toggle) {
      maxSpeed = ElevatorConstants.slowModeSpeed;
    }else{
      maxSpeed = 1;
    }
    slowModeAlert.set(toggle);
  }

  public double getTOF(){
    return TOF.getRange();
  }
  public double getEncoder(){
    return rightMoter.getEncoder().getPosition();
  }
  public boolean getSlowMode(){ //returns true if slow mode is active
    return maxSpeed == ElevatorConstants.slowModeSpeed;
  }
}