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
import frc.robot.Constants.limelightConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Airlock;

public class Elevator extends SubsystemBase {
    private final SparkMax leftMoter, rightMoter;
    private SparkMaxConfig leftConfig, rightConfig;
    PIDController Pid = new PIDController(0.7, 0, 0); 
    PIDController PidSmall = new PIDController(0.3, 0, 0); 
    ElevatorFeedforward feedforward = new ElevatorFeedforward(0, 0.75, 0);
    private TimeOfFlight TOF = new TimeOfFlight(ElevatorConstants.TOFTopCANID);
    Alert leftFaultAlert = new Alert("Faults","", AlertType.kError); 
    Alert rightFaultAlert = new Alert("Faults","", AlertType.kError);
    Alert leftWarningAlert= new Alert("Warnings","", AlertType.kWarning);
    Alert rightWarningAlert = new Alert("Warnings","", AlertType.kWarning);
    double[] L1offset = limelightConstants.LLendoffset; 
    public double targetPos = 15;
    public Double speedMod = 1.0;
    public boolean atMax, atMin, atDrop;
    private Airlock airlock;
    /** Creates a new elevator. */
  public Elevator(Airlock airlock) {
    this.airlock = airlock;
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
    PidSmall.setTolerance(0.05);
    SmartDashboard.putNumber("con",ElevatorConstants.motorRotToIN);
    updateEncoders(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    atMin = getEncoder() <= ElevatorConstants.Min;
    atMax = getEncoder() >= ElevatorConstants.Max;
    atDrop = getEncoder() <= ElevatorConstants.Drop;
    SmartDashboard.putNumber("Elevator/left encoder", getLeftEncoder());
    SmartDashboard.putNumber("Elevator/raw left encoder", leftMoter.getEncoder().getPosition());
    SmartDashboard.putNumber("Elevator/raw right encoder", rightMoter.getEncoder().getPosition());
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
    SmartDashboard.putBoolean("Elevator/Level/L1", getEncoder() >= ElevatorConstants.coralL1-1 && getEncoder() <= ElevatorConstants.coralL1+1);
    SmartDashboard.putBoolean("Elevator/Level/L2", getEncoder() >= ElevatorConstants.coralL2-0.5 && getEncoder() <= ElevatorConstants.coralL2+0.5);
    SmartDashboard.putBoolean("Elevator/Level/L3", getEncoder() >= ElevatorConstants.coralL3-0.5 && getEncoder() <= ElevatorConstants.coralL3+0.5);
    SmartDashboard.putBoolean("Elevator/Level/L4", getEncoder() >= ElevatorConstants.coralL4-0.02 && getEncoder() <= ElevatorConstants.coralL4+0.02);
    leftFaultAlert.setText("elevator left:" + leftMoter.getFaults().toString()); leftFaultAlert.set(leftMoter.hasActiveFault());
    rightFaultAlert.setText("elevator right:" + rightMoter.getFaults().toString()); rightFaultAlert.set(rightMoter.hasActiveFault());
    leftWarningAlert.setText("elevator left" + leftMoter.getWarnings().toString()); leftWarningAlert.set(leftMoter.hasActiveWarning());
    rightWarningAlert.setText("elevator right:" + rightMoter.getWarnings().toString()); rightWarningAlert.set(rightMoter.hasActiveWarning());
  }
  /**sets the speed of the elevator*/
  public void set(double speed){
    if (!airlock.checkSafety()) return;
    if (atMin && speed < 0 || atMax && speed > 0)return; //saftey
    LimelightHelpers.setCameraPose_RobotSpace("limelight-colour", L1offset[0], L1offset[1], L1offset[2]+getEncoder()/39.37, L1offset[3], L1offset[4], L1offset[5]);
    SmartDashboard.putNumber("Elevator/speed", speed*speedMod);
    rightMoter.set(speed*speedMod);
    leftMoter.set(speed*speedMod);
  }
  public void setVoltage(double voltage){
    SmartDashboard.putNumber("Elevator/voltage sent", voltage);
    if (!airlock.checkSafety()) return;
    if (atMin && voltage < 0 || atMax && voltage > 0){System.err.println("ele out of range");return;} //saftey
    rightMoter.setVoltage(voltage);
    leftMoter.setVoltage(voltage);
  }
  /**sets the elevator to a specific position*/
  public void setPID(double setpoint){
    double FF = feedforward.calculate(setpoint);
    double pid = Pid.calculate(getEncoder(), setpoint);
    if(!atDrop && pid > 0) pid += FF;
    if(!atDrop && pid < 0) pid += FF;//-0.4;
    SmartDashboard.putNumber("Elevator/PID/FF", FF);
    SmartDashboard.putNumber("Elevator/PID/target", pid);
    SmartDashboard.putNumber("Elevator/PID/setpoint", setpoint);
    SmartDashboard.putNumber("Elevator/PID/error", Pid.getError());
    setVoltage(pid);
  }
  /**sets the elevator to a specific position*/
  public void setSmallPID(double setpoint){
    double FF = feedforward.calculate(setpoint);
    double pid = PidSmall.calculate(getEncoder(), setpoint);
    if(!atDrop) pid += FF;
    // if(!atDrop && pid < 0) pid += 0.1;
    SmartDashboard.putNumber("Elevator/PID/Small/FF", FF);
    SmartDashboard.putNumber("Elevator/PID/Small/target", pid);
    SmartDashboard.putNumber("Elevator/PID/Small/setpoint", setpoint);
    SmartDashboard.putNumber("Elevator/PID/Small/error", PidSmall.getError());
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
