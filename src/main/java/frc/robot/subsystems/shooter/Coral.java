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
  private final SparkMax leftMotor, rightMotor;
  private SparkMaxConfig rightShooterConfig = new SparkMaxConfig();
  private SparkMaxConfig leftShooterConfig = new SparkMaxConfig();
  private Airlock airlock;
  private Alert leftFaultAlert = new Alert("Faults", "", AlertType.kError);
  private Alert rightFaultAlert = new Alert("Faults", "", AlertType.kError);
  private Alert leftWarningAlert = new Alert("Warnings", "", AlertType.kWarning);
  private Alert rightWarningAlert = new Alert("Warnings", "", AlertType.kWarning);
  /** Creates a new coral thing. */
  public Coral(Airlock airlock) {
    this.airlock = airlock;
    this.leftMotor = new SparkMax(ShooterConstants.leftMotorCANID, MotorType.kBrushless);
    this.rightMotor = new SparkMax(ShooterConstants.rightMotorCANID, MotorType.kBrushless);
    rightShooterConfig.idleMode(IdleMode.kBrake);
    leftShooterConfig.idleMode(IdleMode.kBrake);
    leftShooterConfig.inverted(true);
    leftMotor.configure(leftShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    rightMotor.configure(rightShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter/Left encoder", leftMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Shooter/Right encoder", rightMotor.getEncoder().getPosition());
    leftFaultAlert.setText("coral left:" + leftMotor.getFaults().toString()); leftFaultAlert.set(leftMotor.hasActiveFault());
    rightFaultAlert.setText("coral right:" + rightMotor.getFaults().toString()); rightFaultAlert.set(rightMotor.hasActiveFault());
    leftWarningAlert.setText("coral left:" + leftMotor.getWarnings().toString()); leftWarningAlert.set(leftMotor.hasActiveWarning());
    rightWarningAlert.setText("coral right:" + rightMotor.getWarnings().toString()); rightWarningAlert.set(rightMotor.hasActiveWarning());
    if (airlock.checkStep()) set(0.1);
  }
  public void set(double speed) {
    leftMotor.set(speed);
    rightMotor.set(speed);
  }
  public void stop() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }
} 
