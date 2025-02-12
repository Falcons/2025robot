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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Airlock;

public class Elevator extends SubsystemBase {
    private final SparkMax leftMoter, rightMoter;
    private SparkMaxConfig leftConfig, rightConfig;
    PIDController Pid = new PIDController(0.05, 0.05, 0.05); //TODO: change pid values for elecvator
    private TimeOfFlight TOF = new TimeOfFlight(ElevatorConstants.TOFTopCANID);
    private Alert slowModeAlert = new Alert("Elevator Slow Mode Active", Alert.AlertType.kInfo);
    private double speedMod = 1;
    private Airlock airlock;
    /** Creates a new elevator. */
  public Elevator(Airlock airlock) {
    this.airlock = airlock;
    this.rightMoter = new SparkMax(ElevatorConstants.liftMoter1CANID, MotorType.kBrushless);
    rightConfig = new SparkMaxConfig();
    rightConfig.idleMode(IdleMode.kBrake);
    rightMoter.configure(rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    this.leftMoter = new SparkMax(ElevatorConstants.liftMoter2CANID, MotorType.kBrushless);
    leftConfig = new SparkMaxConfig();
    leftConfig.idleMode(IdleMode.kBrake);
    leftConfig.inverted(true);
    leftMoter.configure(leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    Pid.enableContinuousInput(-180, 180);
    Pid.setTolerance(0.1);
    Pid.setIntegratorRange(-0.01, 0.01);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("left encoder", leftMoter.getEncoder().getPosition());
    SmartDashboard.putNumber("right encoder", rightMoter.getEncoder().getPosition());
    SmartDashboard.putNumber("TOF range", TOF.getRange());
  }
  /**sets the speed of the elevator*/
  public void set(double speed){
    if (!airlock.checkSafety()) return;
    if (getTOF() == ElevatorConstants.TOFMin && speed < 0)return; //if the elevator is at the bottom and the speed is negative, stop the elevator
    if (getTOF() == ElevatorConstants.TOFMax && speed > 0)return; //if the elevator is at the top and the speed is positive, stop the elevator
    rightMoter.set(speed * speedMod);
    leftMoter.set(speed * speedMod);
  }
  public void setPID(double setpoint){
    double encoder = getEncoder();
    rightMoter.set(Pid.calculate(encoder, setpoint));
    leftMoter.set(Pid.calculate(encoder, setpoint));
  }
  /**stops the elevator*/
  public void stop(){
    rightMoter.stopMotor();
    leftMoter.stopMotor();
  }
  /**toggles slow mode*/
  public void setSlowMode(boolean toggle){
    if (toggle) speedMod = ElevatorConstants.slowModeSpeed;
    else speedMod = 1;
    slowModeAlert.set(toggle);
  }
  /**@return the range of the TOF sensor*/
  public double getTOF(){
    return TOF.getRange();
  }
  /**@return the encoder position of the right motor*/
  public double getEncoder(){
    return rightMoter.getEncoder().getPosition();
  }
  /**@return true if slow mode is active*/
  public boolean getSlowMode(){ 
    return speedMod == ElevatorConstants.slowModeSpeed;
  }
  /**@return the level of the elevator*/
  public int getLevel(){
    if(getTOF() > ElevatorConstants.TOFTriggerL1[0] && getTOF() < ElevatorConstants.TOFTriggerL1[1])return 1;
    if(getTOF() > ElevatorConstants.TOFTriggerL2[0] && getTOF() < ElevatorConstants.TOFTriggerL2[1])return 2;
    if(getTOF() > ElevatorConstants.TOFTriggerL3[0] && getTOF() < ElevatorConstants.TOFTriggerL3[1])return 3;
    if(getTOF() > ElevatorConstants.TOFTriggerL4[0] && getTOF() < ElevatorConstants.TOFTriggerL4[1])return 4;
    else return 0;
  }
}
