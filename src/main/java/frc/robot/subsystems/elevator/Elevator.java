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
    PIDController Pid = new PIDController(0.7, 0, 0); //TODO: change pid values for elecvato and FEEDFORWARD
    ElevatorFeedforward feedforward = new ElevatorFeedforward(0, 0.76, 0);
    private TimeOfFlight TOF = new TimeOfFlight(ElevatorConstants.TOFTopCANID);
    private Alert slowModeAlert = new Alert("Elevator Slow Mode Active", AlertType.kInfo);
    Alert leftFaultAlert = new Alert("Faults","", AlertType.kError); 
    Alert rightFaultAlert = new Alert("Faults","", AlertType.kError);
    Alert leftWarningAlert= new Alert("Warnings","", AlertType.kWarning);
    Alert rightWarningAlert = new Alert("Warnings","", AlertType.kWarning);
    private double speedMod = 1;
    public double targetPos = 15;
    public boolean atMax, atMin;
    private Airlock airlock;
    /** Creates a new elevator. */
  public Elevator(Airlock airlock) {
    this.airlock = airlock;
    TOF.setRangingMode(RangingMode.Medium, 24);
    // TOF.setRangeOfInterest(16, 16, 16, 16);
    this.rightMoter = new SparkMax(ElevatorConstants.liftMotor1CANID, MotorType.kBrushless);
    rightConfig = new SparkMaxConfig();
    // rightConfig.encoder.positionConversionFactor(ElevatorConstants.motorRotToIN);
    // rightConfig.encoder.velocityConversionFactor(ElevatorConstants.motorRotToIN);
    rightConfig.idleMode(IdleMode.kBrake);
    rightMoter.configure(rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    this.leftMoter = new SparkMax(ElevatorConstants.liftMotor2CANID, MotorType.kBrushless);
    leftConfig = new SparkMaxConfig();
    leftConfig.encoder.positionConversionFactor(ElevatorConstants.motorRotToIN);
    leftConfig.idleMode(IdleMode.kBrake);
    leftConfig.inverted(true);
    leftMoter.configure(leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    Pid.setTolerance(0.1);
    Pid.setIntegratorRange(-0.01, 0.01);
    updateEncoders(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // updateEncoders(getTofPer()*ElevatorConstants.encoderMax);
    atMin = getTOF() >= ElevatorConstants.TOFMin;
    atMax = getTOF() <= ElevatorConstants.TOFMax;
    SmartDashboard.putNumber("Elevator/left encoder", getLeftEncoder());
    SmartDashboard.putNumber("Elevator/right encoder", getRightEncoder());
    SmartDashboard.putNumber("Elevator/avg encoder", getEncoder());
    SmartDashboard.putNumber("Elevator/left current", getLeftCurent());
    SmartDashboard.putNumber("Elevator/right current", getRightCurent());
    SmartDashboard.putNumber("Elevator/TOF range", getTOF());
    SmartDashboard.putBoolean("Elevator/at max", atMax);
    SmartDashboard.putBoolean("Elevator/at min", atMin);
    leftFaultAlert.setText("elevator left:" + leftMoter.getFaults().toString()); leftFaultAlert.set(leftMoter.hasActiveFault());
    rightFaultAlert.setText("elevator right:" + rightMoter.getFaults().toString()); rightFaultAlert.set(rightMoter.hasActiveFault());
    leftWarningAlert.setText("elevator left" + leftMoter.getWarnings().toString()); leftWarningAlert.set(leftMoter.hasActiveWarning());
    rightWarningAlert.setText("elevator right:" + rightMoter.getWarnings().toString()); rightWarningAlert.set(rightMoter.hasActiveWarning());
  }
  /**sets the speed of the elevator*/
  public void set(double speed){
    if (!airlock.checkSafety()) return;
    if (atMin && speed < 0 || atMax && speed > 0)return; //saftey
    SmartDashboard.putNumber("Elevator/speed", speed);
    rightMoter.set(speed * speedMod);
    leftMoter.set(speed * speedMod);
  }
  public void setVoltage(double voltage){
    SmartDashboard.putNumber("Elevator/voltage sent", voltage);
    if (!airlock.checkSafety()) return;
    if (atMin && voltage < 0 || atMax && voltage > 0)return; //saftey
    rightMoter.setVoltage(voltage);
    leftMoter.setVoltage(voltage);
  }
  /**sets the elevator to a specific position*/
  public void setPID(double setpoint){
    double FF = feedforward.calculate(setpoint);
    setVoltage(-Pid.calculate(getTOF(), setpoint)+ FF);
    SmartDashboard.putNumber("PID/Elevator/FF", FF);
    SmartDashboard.putNumber("PID/Elevator/setpoint", setpoint);
    SmartDashboard.putNumber("PID/Elevator/error", Pid.getError());
  }
  public void resetEncoder(){
    leftMoter.getEncoder().setPosition(0);
    rightMoter.getEncoder().setPosition(0);
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
  /**@return the encoder position of the right motor*/
  public double getRightEncoder(){
    return rightMoter.getEncoder().getPosition()/ElevatorConstants.motorRotToIN;
  }
  /**@return the encoder position of the left motor*/
  public double getLeftEncoder(){
    return leftMoter.getEncoder().getPosition()/ElevatorConstants.motorRotToIN;
  }
  public double getEncoder(){
    double add = getLeftEncoder() + getRightEncoder();
    return add/2.0;
  }
  public void updateEncoders(double value){
    leftMoter.getEncoder().setPosition(value);
    rightMoter.getEncoder().setPosition(value);
  }
  public double getLeftVelocity(){
    return leftMoter.getEncoder().getVelocity()/ElevatorConstants.motorRotToIN;
  }
  public double getRightVelocity(){
    return rightMoter.getEncoder().getVelocity()/ElevatorConstants.motorRotToIN;
  }
  public double getVelocity(){
    double add = getLeftVelocity() + getRightVelocity();
    return add/2.0;
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
}
