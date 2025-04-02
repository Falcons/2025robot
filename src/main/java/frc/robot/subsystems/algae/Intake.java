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

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;

public class Intake extends SubsystemBase {
  private final SparkMax intake;
  private SparkMaxConfig intakeConfig;
  Alert intakeFaultAlert = new Alert("Faults", "", AlertType.kError);
  Alert intakeWarningAlert = new Alert("Warnings", "", AlertType.kWarning);
  double previousCurrent = 0;
  public boolean hasAlgae;
  /** Creates a new intake. */
  public Intake() {
    this.intake = new SparkMax(AlgaeConstants.intakeMotorCANID, MotorType.kBrushless);
    intakeConfig = new SparkMaxConfig();
    intakeConfig.idleMode(IdleMode.kBrake);
    intakeConfig.encoder.positionConversionFactor(AlgaeConstants.intakeMotorRotToRad);
    intakeConfig.encoder.velocityConversionFactor(AlgaeConstants.intakeMotorRotToRad/60);
    intakeConfig.smartCurrentLimit(30);
    intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }
  public void stopIntake() {
    intake.stopMotor();
  }
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Intake/hold", isUnderMinVolts());
    SmartDashboard.putNumber("Intake/Velocity", getVelocity());
    SmartDashboard.putNumber("Intake/Position", getPosition());
    SmartDashboard.putNumber("Intake/current", getCurrent());
    SmartDashboard.putNumber("Intake/bus voltage", getBusVolt());
    intakeFaultAlert.setText("algae intake:" + intake.getFaults().toString()); intakeFaultAlert.set(intake.hasActiveFault());
    intakeWarningAlert.setText("algae intake:" + intake.getFaults().toString()); intakeWarningAlert.set(intake.hasActiveWarning());
  }
  public void setIntake(double speed) {
    intake.set(speed);
  }
  public void setVoltage(double voltage){
    intake.setVoltage(voltage);
  }
  public double getPosition(){
    return intake.getEncoder().getPosition();
  }
  public double getVelocity() {
    return intake.getEncoder().getVelocity();
  }
  public double getCurrent() {
    return intake.getOutputCurrent();
  }
  public double getBusVolt(){
    return intake.getBusVoltage();
  }
  public boolean isUnderMinVolts(){
    hasAlgae = getBusVolt() < AlgaeConstants.voltageMin;
    return hasAlgae;
  }
  // public boolean CurrentSpike(){
  //   if (previousCurrent - getCurrent() > AlgaeConstants.voltageSpikeDifference) {
  //     return true;
  //   }
  //   previousCurrent = getCurrent();
  //   return false;
  // }
}
