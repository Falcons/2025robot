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
    private final SparkMax leftMotor, rightMotor;
    private SparkMaxConfig leftConfig, rightConfig;
    PIDController pid = new PIDController(0.05, 0.05, 0.05); //TODO: change pid values for elecvato and FEEDFORWARD
    private TimeOfFlight TOF = new TimeOfFlight(ElevatorConstants.TOFTopCANID);
    private Alert slowModeAlert = new Alert("Elevator Slow Mode Active", AlertType.kInfo);
    Alert leftFaultAlert = new Alert("Faults","", AlertType.kError); 
    Alert rightFaultAlert = new Alert("Faults","", AlertType.kError);
    Alert leftWarningAlert= new Alert("Warnings","", AlertType.kWarning);
    Alert rightWarningAlert = new Alert("Warnings","", AlertType.kWarning);
    private double speedMod = 1;
    private Airlock airlock;
    ElevatorFeedforward feedforward = new ElevatorFeedforward(0, 0, 0, 0);
    /** Creates a new elevator. */
  public Elevator(Airlock airlock) {
    this.airlock = airlock;
    TOF.setRangingMode(RangingMode.Medium, 24);
    this.rightMotor = new SparkMax(ElevatorConstants.liftMotor1CANID, MotorType.kBrushless);
    rightConfig = new SparkMaxConfig();
    rightConfig.encoder.positionConversionFactor(ElevatorConstants.motorRotToIN);
    rightConfig.idleMode(IdleMode.kBrake);
    rightMotor.configure(rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    this.leftMotor = new SparkMax(ElevatorConstants.liftMotor2CANID, MotorType.kBrushless);
    leftConfig = new SparkMaxConfig();
    leftConfig.encoder.positionConversionFactor(ElevatorConstants.motorRotToIN);
    leftConfig.idleMode(IdleMode.kBrake);
    leftConfig.inverted(true);
    leftMotor.configure(leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    pid.setTolerance(0.1);
    pid.setIntegratorRange(-0.01, 0.01);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // setVoltage(feedforward.calculate(getRightEncoder()));
    SmartDashboard.putNumber("Elevator/left encoder", getLeftEncoder());
    SmartDashboard.putNumber("Elevator/left current", getLeftCurrent());
    SmartDashboard.putNumber("Elevator/right encoder", getRightEncoder());
    SmartDashboard.putNumber("Elevator/right current", getRightCurrent());
    SmartDashboard.putNumber("Elevator/TOF range", TOF.getRange());
    leftFaultAlert.setText("elevator left:" + leftMotor.getFaults().toString()); leftFaultAlert.set(leftMotor.hasActiveFault());
    rightFaultAlert.setText("elevator right:" + rightMotor.getFaults().toString()); rightFaultAlert.set(rightMotor.hasActiveFault());
    leftWarningAlert.setText("elevator left" + leftMotor.getWarnings().toString()); leftWarningAlert.set(leftMotor.hasActiveWarning());
    rightWarningAlert.setText("elevator right:" + rightMotor.getWarnings().toString()); rightWarningAlert.set(rightMotor.hasActiveWarning());
  }
  /**sets the speed of the elevator*/
  public void set(double speed){
    
    if (!airlock.checkSafety()) return;
    /*
    if (getTOF() == ElevatorConstants.TOFMin && speed < 0)return; //if the elevator is at the bottom and the speed is negative, stop the elevator
    if (getTOF() == ElevatorConstants.TOFMax && speed > 0)return; //if the elevator is at the top and the speed is positive, stop the elevator
    */

    rightMotor.set(speed * speedMod);
    leftMotor.set(speed * speedMod);
  }
  public void setVoltage(double voltage){
    rightMotor.setVoltage(voltage);
    leftMotor.setVoltage(voltage);
  }
  /**sets the elevator to a specific position*/
  public void setPID(double setpoint){
    setVoltage(pid.calculate(getRightEncoder(), setpoint) + feedforward.calculate(setpoint));
  }
  /**stops the elevator*/
  public void stop(){
    rightMotor.stopMotor();
    leftMotor.stopMotor();
  }
  /**toggles slow mode*/
  public void setSlowMode(boolean toggle){
    if (toggle) speedMod = ElevatorConstants.slowModeSpeed;
    else speedMod = 1;
    slowModeAlert.set(toggle);
  }
  public boolean atSetpoint(){
    return pid.atSetpoint();
  }
  /**@return the range of the TOF sensor in inchs*/
  public double getTOF(){
    return TOF.getRange()/25.4;
  }
  /**@return the encoder position of the right motor*/
  public double getRightEncoder(){
    return rightMotor.getEncoder().getPosition();
  }
  /**@return the encoder position of the left motor*/
  public double getLeftEncoder(){
    return leftMotor.getEncoder().getPosition();
  }
  public double getVelocity(){
    return rightMotor.getEncoder().getVelocity();
  }
  public double getLeftCurrent(){
    return leftMotor.getOutputCurrent();
  }
  public double getRightCurrent(){
    return rightMotor.getOutputCurrent();
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
