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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;

public class Algae extends SubsystemBase {
  private final SparkMax pivot, intake;
  private SparkMaxConfig pivotConfig, intakeConfig;
  PIDController pivotPid = new PIDController(0.05, 0.05, 0.05); //TODO: change pid values for algae
  double pivotAngle = 0;
  /** Creates a new algea_pivot. */
  public Algae() {
    this.pivot = new SparkMax(AlgaeConstants.pivotMoterCANID, MotorType.kBrushless);
    this.intake = new SparkMax(AlgaeConstants.intakeMoterCANID, MotorType.kBrushless);
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

    pivotAngle = pivot.getEncoder().getPosition();
  }
  public void pidReset() {
    pivotPid.reset();
  }
  public void setPivot(double speed) {
    pivot.set(speed);
  }
  public void setIntake(double speed) {
    intake.set(speed);
  }
  public void setPivotpid(double speed){ //TODO: change encoder value to an angle
    pivot.set(pivotPid.calculate(pivotAngle, AlgaeConstants.algeaPivotSetpoint));
  }
}
