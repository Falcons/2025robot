// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Airlock;

public class Coral extends SubsystemBase {
  private final SparkMax leftShooter, rightShooter;
  private SparkMaxConfig shooterConfig = new SparkMaxConfig();
  private Alert leftFaultAlert, rightFaultAlert = new Alert("Faults","", AlertType.kError);
  private Alert leftWarningAlert, rightWarningAlert = new Alert("Warnings","", AlertType.kWarning);
  /** Creates a new coral thing. */
  public Coral(Airlock airlock) {
    this.leftShooter = new SparkMax(ShooterConstants.leftMoterCANID, MotorType.kBrushless);
    this.rightShooter = new SparkMax(ShooterConstants.rightMoterCANID, MotorType.kBrushless);
      shooterConfig.idleMode(IdleMode.kBrake);
    leftShooter.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    rightShooter.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }
  public void stop() {
    leftShooter.stopMotor();
    rightShooter.stopMotor();
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left shooter encoder", leftShooter.getEncoder().getPosition());
    SmartDashboard.putNumber("Right shooter encoder", rightShooter.getEncoder().getPosition());
    if(leftShooter.hasActiveFault()) leftFaultAlert.setText("coral left:" + leftShooter.getFaults().toString()); leftFaultAlert.set(true);
    if(rightShooter.hasActiveFault()) rightFaultAlert.setText("coral right:" + rightShooter.getFaults().toString()); rightFaultAlert.set(true);
    if(leftShooter.hasActiveWarning()) leftWarningAlert.setText("coral left:" + leftShooter.getWarnings().toString()); leftWarningAlert.set(true);
    if(rightShooter.hasActiveWarning()) rightWarningAlert.setText("coral right:" + rightShooter.getWarnings().toString()); rightWarningAlert.set(true);
  }
  public void set(double speed) {
    leftShooter.set(speed);
    rightShooter.set(speed);
  }
} 
