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
  PIDController pivotPid = new PIDController(0.05, 0.05, 0.05); //TODO: change pid values for algae
  ArmFeedforward feedforward = new ArmFeedforward(0.05, 0.05, 0.05); //TODO: change feedforward values for algaes

  Alert pivotFaultAlert = new Alert("Faults", "", AlertType.kError);
  Alert pivotWarningAlert = new Alert("Warnings", "", AlertType.kWarning);
  double previousCurrent = 0;
  /** Creates a new algea_pivot. */
  public Pivot() {
    this.pivot = new SparkMax(AlgaeConstants.pivotMoterCANID, MotorType.kBrushless);
    pivotConfig = new SparkMaxConfig();
    pivotConfig.idleMode(IdleMode.kBrake);
    pivotConfig.encoder.positionConversionFactor(AlgaeConstants.pivotMoterRotToDegree);

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
    SmartDashboard.putNumber("Pivot/Encoder", pivot.getEncoder().getPosition());
    pivotFaultAlert.setText("algea pivot:" + pivot.getFaults().toString()); pivotFaultAlert.set(pivot.hasActiveFault());
    pivotWarningAlert.setText("algea pivot:" + pivot.getFaults().toString()); pivotWarningAlert.set(pivot.hasActiveWarning());
  }
  public void pidReset() {
    pivotPid.reset();
  }
  public void setPivot(double speed) {
    if (!currentSpike()) {
      pivot.set(speed);
    }
  }
  public void setPivotpid(double level) {
    pivot.set(pivotPid.calculate(pivot.getEncoder().getPosition(), level));
  }
  /**
   * @param p position
   * @param v volocity
   * @param a acceleration
   */
  public void setPivotFeedFowerd(double p, double v,Double a) {
    pivot.setVoltage(feedforward.calculate(p, v, a));
  }

  public double getPivotPos() {
    return pivot.getEncoder().getPosition();
  }
  public double getPivotCurrent() {
    return pivot.getOutputCurrent();
  }
  public boolean currentSpike(){
    if (previousCurrent - getPivotCurrent() > AlgaeConstants.voltageSpikeDifference) {
      return true;
    }
    previousCurrent = getPivotCurrent();
    return false;
  }

  public boolean atSetpoint(){
    return pivotPid.atSetpoint();
  }
}
