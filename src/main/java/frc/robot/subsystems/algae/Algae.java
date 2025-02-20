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

public class Algae extends SubsystemBase {
  private final SparkMax pivot, intake;
  private SparkMaxConfig pivotConfig, intakeConfig;
  PIDController pivotPid = new PIDController(0.05, 0.05, 0.05); //TODO: change pid values for algae
  ArmFeedforward feedforward = new ArmFeedforward(0.05, 0.05, 0.05); //TODO: change feedforward values for algaes
  double pivotAngle = 0;

  Alert pivotFaultAlert = new Alert("Faults", "", AlertType.kError);
  Alert intakeFaultAlert = new Alert("Faults", "", AlertType.kError);
  Alert pivotWarningAlert = new Alert("Warnings", "", AlertType.kWarning);
  Alert intakeWarningAlert = new Alert("Warnings", "", AlertType.kWarning);
  double previousCurrent = 0;
  /** Creates a new algea_pivot. */
  public Algae() {
    this.pivot = new SparkMax(AlgaeConstants.pivotMoterCANID, MotorType.kBrushless);
    this.intake = new SparkMax(AlgaeConstants.intakeMoterCANID, MotorType.kBrushless);
    pivotConfig = new SparkMaxConfig();
    intakeConfig = new SparkMaxConfig();
    pivotConfig.idleMode(IdleMode.kBrake);
    intakeConfig.encoder.positionConversionFactor(AlgaeConstants.pivotMotorRotToDegree);//distance per pulse

    pivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    pivotPid.enableContinuousInput(-180, 180);
    pivotPid.setTolerance(0.1);
    pivotPid.setIntegratorRange(-0.01, 0.01);
  }
  public void stopPivot() {
    pivot.stopMotor();
  }
  public void stopIntake() {
    intake.stopMotor();
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pivot Encoder", pivot.getEncoder().getPosition());
    SmartDashboard.putNumber("Intake Encoder", intake.getEncoder().getPosition());
    pivotFaultAlert.setText("algea pivot:" + pivot.getFaults().toString()); pivotFaultAlert.set(pivot.hasActiveFault());
    intakeFaultAlert.setText("algea intake:" + intake.getFaults().toString()); intakeFaultAlert.set(intake.hasActiveFault());
    pivotWarningAlert.setText("algea pivot:" + pivot.getFaults().toString()); pivotWarningAlert.set(pivot.hasActiveWarning());
    intakeWarningAlert.setText("algea intake:" + intake.getFaults().toString()); intakeWarningAlert.set(intake.hasActiveWarning());
  }
  public void pidReset() {
    pivotPid.reset();
  }
  public void setPivot(double speed) {
    if (!currentSpike()) {
      pivot.set(speed);
    }
  }
  public void setIntake(double speed) {
    intake.set(speed);
  }
  public void setPivotpid(double angle) {
    pivotAngle = pivot.getEncoder().getPosition();
    pivot.set(pivotPid.calculate(pivotAngle, angle));
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
  public double getIntakeVel() {
    return intake.getEncoder().getVelocity();
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
}
