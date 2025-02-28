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

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;

public class Pivot extends SubsystemBase {
  private final SparkMax pivot;
  private SparkMaxConfig pivotConfig;
  PIDController pivotPID = new PIDController(0.05, 0.05, 0.05); //TODO: change pid values for algae
  ArmFeedforward feedforward = new ArmFeedforward(0.05, 0.05, 0.05,0.05); //TODO: change feedforward values for algaes

  Alert pivotFaultAlert = new Alert("Faults", "", AlertType.kError);
  Alert pivotWarningAlert = new Alert("Warnings", "", AlertType.kWarning);
  double previousCurrent = 0;
  boolean pivotTop = true;
  /** Creates a new algae_pivot. */
  public Pivot() {
    this.pivot = new SparkMax(AlgaeConstants.pivotMotorCANID, MotorType.kBrushless);
    pivotConfig = new SparkMaxConfig();
    pivotConfig.idleMode(IdleMode.kBrake);
    pivotConfig.encoder.positionConversionFactor(AlgaeConstants.pivotMotorRotToDegree);

    pivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    pivotPID.enableContinuousInput(-180, 180);
    pivotPID.setTolerance(0.1);
    pivotPID.setIntegratorRange(-0.01, 0.01);
  }
  public void stopPivot() {
    pivot.stopMotor();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pivot/Encoder", getPivotPos());
    SmartDashboard.putNumber("Pivot/Current", getPivotCurrent());
    pivotFaultAlert.setText("Pivot/algae pivot:" + pivot.getFaults().toString()); pivotFaultAlert.set(pivot.hasActiveFault());
    pivotWarningAlert.setText("Pivot/algae pivot:" + pivot.getFaults().toString()); pivotWarningAlert.set(pivot.hasActiveWarning());
  }
  public void pidReset() {
    pivotPID.reset();
  }
  public void setPivot(double speed) {
    // if (!currentSpike()) {
      pivot.set(speed);
    // }
  }
  // public void setPivotpid(double angle) {
  //   pivot.set(pivotPID.calculate(getPivotPos(), angle) + feedforward.calculate(angle, 0));
  // }
  public void togglePivot(){
    pivotTop = !pivotTop;
    if (pivotTop) {
      pivot.set(pivotPID.calculate(getPivotPos(), AlgaeConstants.pivotMotorBottom) + feedforward.calculate(AlgaeConstants.pivotMotorBottom, 0));
      pivotTop = false;
    } else {
      pivot.set(pivotPID.calculate(getPivotPos(), AlgaeConstants.pivotMotorTop) + feedforward.calculate(AlgaeConstants.pivotMotorTop, 0));
      pivotTop = true;
    }
  }
  /**
   * @param p position
   * @param v volocity
   * @param a acceleration
   */
  public void setPivotFeedFowerd(double p, double v) {
    pivot.setVoltage(feedforward.calculate(p, v));
  }

  public double getPivotPos() {
    return pivot.getEncoder().getPosition();
  }
  public double getPivotCurrent() {
    return pivot.getOutputCurrent();
  }
  public boolean currentSpike(){
    if (previousCurrent - getPivotCurrent() >= AlgaeConstants.voltageSpikeDifference) {
      return true;
    }
    previousCurrent = getPivotCurrent();
    return false;
  }

  public boolean atSetpoint(){
    return pivotPID.atSetpoint();
  }
}
