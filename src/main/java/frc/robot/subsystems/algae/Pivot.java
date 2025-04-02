// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algae;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.subsystems.elevator.Elevator;

public class Pivot extends SubsystemBase {
  private final SparkMax pivot;
  private SparkMaxConfig pivotConfig;
  PIDController pivotPid = new PIDController(0.3, 0, 0);

  Alert pivotFaultAlert = new Alert("Faults", "", AlertType.kError);
  Alert pivotWarningAlert = new Alert("Warnings", "", AlertType.kWarning);
  double previousCurrent = 0;
  public boolean atMin, atMax;
  Elevator elevator;
  /** Creates a new algea_pivot. */
  public Pivot(Elevator elevator) {
    this.elevator = elevator;
    this.pivot = new SparkMax(AlgaeConstants.pivotMotorCANID, MotorType.kBrushless);
    pivotConfig = new SparkMaxConfig();
    pivotConfig.idleMode(IdleMode.kBrake);
    pivotConfig.encoder.positionConversionFactor(AlgaeConstants.pivotMotorRotToRad);
    pivotConfig.encoder.velocityConversionFactor(AlgaeConstants.pivotMotorRotToRad);
    pivotConfig.smartCurrentLimit(30);
    pivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    pivotPid.enableContinuousInput(-180, 180);
    pivotPid.setTolerance(0.05);
    pivotPid.setIntegratorRange(-0.01, 0.01);
  }
  public void stopPivot() {
    pivot.stopMotor();
  }

  @Override
  public void periodic() {
    elevator.danger = getAbsEncoderDeg() < AlgaeConstants.pivotDanger &&  elevator.getEncoder() <= AlgaeConstants.MaxAlgaeHeight;
    atMax = getAbsEncoderDeg() >= AlgaeConstants.pivotMax;
    atMin = getAbsEncoderDeg() <= AlgaeConstants.pivotMin;

    SmartDashboard.putNumber("Pivot/PID/error", pivotPid.getError());
    SmartDashboard.putNumber("Pivot/PID/setpoint", pivotPid.getSetpoint());
    SmartDashboard.putNumber("Pivot/Abs Encoder", getAbsolute());
    SmartDashboard.putNumber("Pivot/Abs Encoder deg", getAbsEncoderDeg());
    SmartDashboard.putNumber("Pivot/current", getCurrent());
    SmartDashboard.putNumber("Pivot/bus voltage", getBusVoltage());
    SmartDashboard.putBoolean("Pivot/at max", atMax);
    SmartDashboard.putBoolean("Pivot/at min", atMin);
    pivotFaultAlert.setText("algae pivot:" + pivot.getFaults().toString()); pivotFaultAlert.set(pivot.hasActiveFault());
    pivotWarningAlert.setText("algae pivot:" + pivot.getFaults().toString()); pivotWarningAlert.set(pivot.hasActiveWarning());
    pidReset();
  }
  public void pidReset() {
    pivotPid.reset();
  }
  public void setPivot(double speed) {
    if(atMax && speed > 0){speed = 0; stopPivot();}
    if(atMin && speed < 0){speed = 0; stopPivot();}
    SmartDashboard.putNumber("Pivot/speed", speed);
    pivot.set(speed);
  }
  public void setPivotpid(double setpoint) {
    double pid = pivotPid.calculate(getAbsolute(), setpoint);
    if(setpoint > AlgaeConstants.pivotMax) setpoint = AlgaeConstants.pivotMax;
    if(setpoint < AlgaeConstants.pivotMin) setpoint = AlgaeConstants.pivotMin;
    if(pid > 0.5) pid = 0.5;
    if(pid < -0.5) pid = -0.5;
    SmartDashboard.putNumber("Pivot/PID/error", pivotPid.getError());
    SmartDashboard.putNumber("Pivot/PID/setpoint", setpoint);
    SmartDashboard.putNumber("Pivot/PID/calc", pid);

    setPivot(pid);
  }
  public double getReletive() {
    return pivot.getEncoder().getPosition();
  }
  public double getAbsolute(){
    return pivot.getAbsoluteEncoder().getPosition()*2 * Math.PI - 2.431;
  }
  public double getAbsEncoderDeg(){
    return getAbsolute()*180.0/Math.PI;
  }
  public double getCurrent() {
    return pivot.getOutputCurrent();
  }
  public double getBusVoltage(){
    return pivot.getBusVoltage();
  }
  public boolean atSetpoint(){
    return pivotPid.atSetpoint();
  }
}
