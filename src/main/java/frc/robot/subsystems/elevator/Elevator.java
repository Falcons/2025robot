// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.limelightConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Airlock;
import frc.robot.subsystems.FalconFlare;

public class Elevator extends SubsystemBase {
    private final SparkMax leftMoter, rightMoter;
    private SparkMaxConfig leftConfig, rightConfig;
    PIDController Pid = new PIDController(0.7, 0, 0);  //0.7, 0, 0
    ElevatorFeedforward feedforwardMid = new ElevatorFeedforward(0, ElevatorConstants.FFMid, 0); //0 , mid , 0
    ElevatorFeedforward feedforwardHigh = new ElevatorFeedforward(0, ElevatorConstants.FFhigh, 0); // 0 , high, 0
    private TimeOfFlight TOF = new TimeOfFlight(ElevatorConstants.TOFTopCANID);
    Alert leftFaultAlert = new Alert("Faults","", AlertType.kError); 
    Alert rightFaultAlert = new Alert("Faults","", AlertType.kError);
    Alert leftWarningAlert= new Alert("Warnings","", AlertType.kWarning);
    Alert rightWarningAlert = new Alert("Warnings","", AlertType.kWarning);
    double[] L1offset = limelightConstants.LLendoffset; 
    public double targetPos = 15;
    public Double speedMod = 1.0;
    public boolean atMax, atMin, atDrop, danger;
    private Airlock airlock;
    FalconFlare falconFlare;
    /** Creates a new elevator. */
  public Elevator(Airlock airlock, FalconFlare falconFlare) {
    this.airlock = airlock;
    this.falconFlare = falconFlare;
    TOF.setRangingMode(RangingMode.Short, 24);
    this.rightMoter = new SparkMax(ElevatorConstants.liftMotor1CANID, MotorType.kBrushless);
    rightConfig = new SparkMaxConfig();
    rightConfig.encoder.positionConversionFactor(2);
    rightConfig.encoder.velocityConversionFactor(2);
    rightConfig.idleMode(IdleMode.kBrake);
    rightConfig.inverted(false);
    rightConfig.smartCurrentLimit(40);
    rightMoter.configure(rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    this.leftMoter = new SparkMax(ElevatorConstants.liftMotor2CANID, MotorType.kBrushless);
    leftConfig = new SparkMaxConfig();
    leftConfig.encoder.positionConversionFactor(2);
    leftConfig.encoder.velocityConversionFactor(2);
    leftConfig.idleMode(IdleMode.kBrake);
    leftConfig.inverted(true);
    leftConfig.smartCurrentLimit(40);
    leftMoter.configure(leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    Pid.setTolerance(0.05);
    Pid.setIntegratorRange(-0.01, 0.01);
    updateEncoders(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    atMin = getEncoder() <= ElevatorConstants.Min;
    atMax = getEncoder() >= ElevatorConstants.Max;
    atDrop = getEncoder() <= ElevatorConstants.Drop;
    SmartDashboard.putNumber("Elevator/left encoder", getLeftEncoder());
    SmartDashboard.putNumber("Elevator/right encoder", getRightEncoder());
    SmartDashboard.putNumber("Elevator/avg encoder", getEncoder());
    SmartDashboard.putNumber("Elevator/left velocity", getLeftVelocity());
    SmartDashboard.putNumber("Elevator/right velocity", getRightVelocity());
    SmartDashboard.putNumber("Elevator/avg velocity", getVelocity());
    SmartDashboard.putNumber("Elevator/left current", getLeftCurent());
    SmartDashboard.putNumber("Elevator/right current", getRightCurent());
    SmartDashboard.putNumber("Elevator/TOF range", getTOF());
    SmartDashboard.putBoolean("Elevator/at max", atMax);
    SmartDashboard.putBoolean("Elevator/at min", atMin);
    SmartDashboard.putBoolean("Elevator/at drop", atDrop);
    SmartDashboard.putBoolean("Elevator/in danger", danger);
    SmartDashboard.putBoolean("Elevator/Level/L1", getEncoder() >= ElevatorConstants.coralL1-0.5 && getEncoder() <= ElevatorConstants.coralL1+0.5);
    SmartDashboard.putBoolean("Elevator/Level/L2", getEncoder() >= ElevatorConstants.coralL2-0.5 && getEncoder() <= ElevatorConstants.coralL2+0.5);
    SmartDashboard.putBoolean("Elevator/Level/L3", getEncoder() >= ElevatorConstants.coralL3-0.5 && getEncoder() <= ElevatorConstants.coralL3+0.5);
    SmartDashboard.putBoolean("Elevator/Level/L4", getEncoder() >= ElevatorConstants.coralL4L-0.02 && getEncoder() <= ElevatorConstants.coralL4H+0.02);
    leftFaultAlert.setText("elevator left:" + leftMoter.getFaults().toString()); leftFaultAlert.set(leftMoter.hasActiveFault());
    rightFaultAlert.setText("elevator right:" + rightMoter.getFaults().toString()); rightFaultAlert.set(rightMoter.hasActiveFault());
    leftWarningAlert.setText("elevator left" + leftMoter.getWarnings().toString()); leftWarningAlert.set(leftMoter.hasActiveWarning());
    rightWarningAlert.setText("elevator right:" + rightMoter.getWarnings().toString()); rightWarningAlert.set(rightMoter.hasActiveWarning());
    if(!DriverStation.isDisabled()){
      if (getEncoder() >= ElevatorConstants.coralL1-0.5 && getEncoder() <= ElevatorConstants.coralL1+0.5){falconFlare.setLights("green");}
      if (getEncoder() >= ElevatorConstants.coralL2-0.5 && getEncoder() <= ElevatorConstants.coralL2+0.5){falconFlare.setLights("blue");}
      if (getEncoder() >= ElevatorConstants.coralL3-0.5 && getEncoder() <= ElevatorConstants.coralL3+0.5){falconFlare.setLights("purple");}
      if (getEncoder() >= ElevatorConstants.coralL4L-0.02 && getEncoder() <= ElevatorConstants.coralL4H+0.02){falconFlare.setLights("red");}
      if (getEncoder() >= ElevatorConstants.algaeL2-0.1 && getEncoder() <= ElevatorConstants.algaeL2+0.1){falconFlare.setLights("yellow");}
      if (getEncoder() >= ElevatorConstants.algaeL3-0.1 && getEncoder() <= ElevatorConstants.algaeL3+0.1){falconFlare.setLights("yellow");}
      if (getEncoder() < ElevatorConstants.coralL1-0.5) {falconFlare.setLights("white");}
    }
  }
  /**sets the speed of the elevator*/
  public void set(double speed){
    if (!airlock.checkSafety()) speed = 0;
    if (danger && speed < 0)speed = 0;
    if (atMin && speed < 0 || atMax && speed > 0)speed = 0; //saftey
    LimelightHelpers.setCameraPose_RobotSpace("limelight-end", L1offset[0], L1offset[1], L1offset[2]+getEncoder()/78.74, L1offset[3], L1offset[4], L1offset[5]);
    SmartDashboard.putNumber("Elevator/speed", speed*speedMod);
    rightMoter.set(speed*speedMod);
    leftMoter.set(speed*speedMod);
  }
  public void setVoltage(double voltage){
    SmartDashboard.putNumber("Elevator/voltage sent", voltage);
    if (!airlock.checkSafety()) return;
    LimelightHelpers.setCameraPose_RobotSpace("limelight-end", L1offset[0], L1offset[1], L1offset[2]+getEncoder()/78.74, L1offset[3], L1offset[4], L1offset[5]);
    if (atMin && voltage < 0 || atMax && voltage > 0){System.err.println("ele out of range");return;} //saftey
    rightMoter.setVoltage(voltage);
    leftMoter.setVoltage(voltage);
  }
  /**sets the elevator to a specific position*/
  public void setPID(double setpoint){
    double FFMid = feedforwardMid.calculate(setpoint);
    double FFHigh = feedforwardHigh.calculate(setpoint);
    double pid = Pid.calculate(getEncoder(), setpoint);
    if(!atDrop && getEncoder() >= 119) pid += FFHigh;
    else if(!atDrop) pid += FFMid;
    SmartDashboard.putNumber("Elevator/PID/FF Mid", FFMid);
    SmartDashboard.putNumber("Elevator/PID/FF high", FFHigh);
    SmartDashboard.putNumber("Elevator/PID/target", pid);
    SmartDashboard.putNumber("Elevator/PID/setpoint", setpoint);
    SmartDashboard.putNumber("Elevator/PID/error", Pid.getError());
    setVoltage(pid);
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
  public boolean atSetpoint(){
    return Pid.atSetpoint();
  }
  /**@return the range of the TOF sensor in inchs*/
  public double getTOF(){
    return TOF.getRange();
  }
  /**@return the encoder position of the right motor*/
  public double getRightEncoder(){
    return rightMoter.getEncoder().getPosition();
  }
  /**@return the encoder position of the left motor*/
  public double getLeftEncoder(){
    return leftMoter.getEncoder().getPosition();
  }
  public double getEncoder(){
    double add = getLeftEncoder() + getRightEncoder();
    return add/2.0;
  }
  public void updateEncoders(double value){
    leftMoter.getEncoder().setPosition(value);
    rightMoter.getEncoder().setPosition(value);
    System.out.println("ele encoders set to " + value);
  }
  public double getLeftVelocity(){
    return leftMoter.getEncoder().getVelocity();
  }
  public double getRightVelocity(){
    return rightMoter.getEncoder().getVelocity();
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
}
