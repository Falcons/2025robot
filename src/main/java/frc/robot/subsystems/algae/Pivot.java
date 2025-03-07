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

public class Pivot extends SubsystemBase {
  private final SparkMax pivot;
  private SparkMaxConfig pivotConfig;
  PIDController pivotPid = new PIDController(0.05, 0, 0); //TODO: change pid values for algae //0.5 0 0

  Alert pivotFaultAlert = new Alert("Faults", "", AlertType.kError);
  Alert pivotWarningAlert = new Alert("Warnings", "", AlertType.kWarning);
  double previousCurrent = 0;
  public boolean atMin, atMax, isOut;
  /** Creates a new algea_pivot. */
  public Pivot() {
    this.pivot = new SparkMax(AlgaeConstants.pivotMotorCANID, MotorType.kBrushless);
    pivotConfig = new SparkMaxConfig();
    // pivotConfig.idleMode(IdleMode.kBrake);
    pivotConfig.idleMode(IdleMode.kBrake);
    pivotConfig.encoder.positionConversionFactor(AlgaeConstants.pivotMotorRotToRad);
    pivotConfig.encoder.velocityConversionFactor(AlgaeConstants.pivotMotorRotToRad);

    pivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    pivotPid.enableContinuousInput(-180, 180);
    pivotPid.setTolerance(0.1);
    pivotPid.setIntegratorRange(-0.01, 0.01);
  }
  public void stopPivot() {
    pivot.stopMotor();
  }

  @Override
  public void periodic() {
    atMax = getAbsEncoderDeg() >= AlgaeConstants.pivotMax;
    atMin = getAbsEncoderDeg() <= AlgaeConstants.pivotMin;
    isOut = getAbsEncoderDeg() >= AlgaeConstants.pivotOut-1 && getAbsEncoderDeg() <= AlgaeConstants.pivotOut+1;

    SmartDashboard.putNumber("Pivot/PID/error", pivotPid.getError());
    SmartDashboard.putNumber("Pivot/PID/setpoint", pivotPid.getSetpoint());
    SmartDashboard.putNumber("Pivot/Abs Encoder", getAbsolute());
    SmartDashboard.putNumber("Pivot/Abs Encoder deg", getAbsEncoderDeg());
    SmartDashboard.putNumber("Pivot/current", getCurrent());
    SmartDashboard.putBoolean("Pivot/at max", atMax);
    SmartDashboard.putBoolean("Pivot/at min", atMin);
    pivotFaultAlert.setText("algae pivot:" + pivot.getFaults().toString()); pivotFaultAlert.set(pivot.hasActiveFault());
    pivotWarningAlert.setText("algae pivot:" + pivot.getFaults().toString()); pivotWarningAlert.set(pivot.hasActiveWarning());
  }
  public void pidReset() {
    pivotPid.reset();
  }
  public void setPivot(double speed) {
      if(atMax && speed > 0) return;
      if(atMin && speed < 0) return;
      SmartDashboard.putNumber("Pivot/speed", speed);
      pivot.set(speed);
  }
  public void setPivotpid(double setpoint) {
    double pid = pivotPid.calculate(getAbsolute(), setpoint);
    SmartDashboard.putNumber("Pivot/PID/error", pivotPid.getError());
    SmartDashboard.putNumber("Pivot/PID/setpoint", pid);
    SmartDashboard.putNumber("Pivot/PID/calc", pid);
    // if(pid > AlgaeConstants.pivotMax) pid = 0;
    // if(pid < AlgaeConstants.pivotMin) pid = 0;
    setPivot(pid);
  }
  /**
   * @param p position
   * @param v volocity
   * @param a acceleration
   */

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
  public boolean atSetpoint(){
    return pivotPid.atSetpoint();
  }
}
