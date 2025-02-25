// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Airlock;

public class Elevator extends SubsystemBase {
    private final SparkMax leftMoter, rightMoter;
    private SparkMaxConfig leftConfig, rightConfig;
    PIDController Pid = new PIDController(0.05, 0.05, 0.05); //TODO: change pid values for elecvator
    private TimeOfFlight TOF = new TimeOfFlight(ElevatorConstants.TOFTopCANID);
    private Alert slowModeAlert = new Alert("Elevator Slow Mode Active", AlertType.kInfo);
    Alert leftFaultAlert = new Alert("Faults","", AlertType.kError); 
    Alert rightFaultAlert = new Alert("Faults","", AlertType.kError);
    Alert leftWarningAlert= new Alert("Warnings","", AlertType.kWarning);
    Alert rightWarningAlert = new Alert("Warnings","", AlertType.kWarning);
    private double speedMod = 1;
    private Airlock airlock;
    /** Creates a new elevator. */
  public Elevator(Airlock airlock) {
    this.airlock = airlock;
    TOF.setRangingMode(RangingMode.Medium, 24);
    this.rightMoter = new SparkMax(ElevatorConstants.liftMoter1CANID, MotorType.kBrushless);
    rightConfig = new SparkMaxConfig();
    rightConfig.encoder.positionConversionFactor(ElevatorConstants.moterRotToIN);
    rightConfig.idleMode(IdleMode.kBrake);
    rightMoter.configure(rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    this.leftMoter = new SparkMax(ElevatorConstants.liftMoter2CANID, MotorType.kBrushless);
    leftConfig = new SparkMaxConfig();
    leftConfig.encoder.positionConversionFactor(ElevatorConstants.moterRotToIN);
    leftConfig.idleMode(IdleMode.kBrake);
    leftConfig.inverted(true);
    leftMoter.configure(leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    Pid.setTolerance(0.1);
    Pid.setIntegratorRange(-0.01, 0.01);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator/left encoder", leftMoter.getEncoder().getPosition());
    SmartDashboard.putNumber("Elevator/left current", leftMoter.getOutputCurrent());
    SmartDashboard.putNumber("Elevator/right encoder", rightMoter.getEncoder().getPosition());
    SmartDashboard.putNumber("Elevator/right current", rightMoter.getOutputCurrent());
    SmartDashboard.putNumber("Elevator/TOF range", TOF.getRange());
    leftFaultAlert.setText("elevator left:" + leftMoter.getFaults().toString()); leftFaultAlert.set(leftMoter.hasActiveFault());
    rightFaultAlert.setText("elevator right:" + rightMoter.getFaults().toString()); rightFaultAlert.set(rightMoter.hasActiveFault());
    leftWarningAlert.setText("elevator left" + leftMoter.getWarnings().toString()); leftWarningAlert.set(leftMoter.hasActiveWarning());
    rightWarningAlert.setText("elevator right:" + rightMoter.getWarnings().toString()); rightWarningAlert.set(rightMoter.hasActiveWarning());
  }
  /**sets the speed of the elevator*/
  public void set(double speed){
    if (!airlock.checkSafety()) return;
    if (getTOF() == ElevatorConstants.TOFMin && speed < 0)return; //if the elevator is at the bottom and the speed is negative, stop the elevator
    if (getTOF() == ElevatorConstants.TOFMax && speed > 0)return; //if the elevator is at the top and the speed is positive, stop the elevator
    rightMoter.set(speed * speedMod);
    leftMoter.set(speed * speedMod);
  }
  /**sets the elevator to a specific position*/
  public void setPID(double setpoint){
    set(Pid.calculate(getEncoder(), setpoint));
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
  public boolean atSetpoint(){
    return Pid.atSetpoint();
  }
  /**@return the range of the TOF sensor in inchs*/
  public double getTOF(){
    return TOF.getRange()/25.4;
  }
  /**@return the encoder position of the left motor*/
  public double getEncoder(){
    return rightMoter.getEncoder().getPosition();
  }
  public double getVelocity(){
    return rightMoter.getEncoder().getVelocity();
  }
  public double getLeftCurent(){
    return leftMoter.getOutputCurrent();
  }
  public double getRightCurent(){
    return rightMoter.getOutputCurrent();
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
