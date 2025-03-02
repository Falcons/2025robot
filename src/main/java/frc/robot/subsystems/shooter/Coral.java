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
    this.leftShooter = new SparkMax(ShooterConstants.leftMotorCANID, MotorType.kBrushless);
    this.rightShooter = new SparkMax(ShooterConstants.rightMotorCANID, MotorType.kBrushless);
    rightShooterConfig.idleMode(IdleMode.kBrake);
    leftShooterConfig.idleMode(IdleMode.kBrake);
    leftShooterConfig.inverted(true);
    leftShooter.configure(leftShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    rightShooter.configure(rightShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter/Left encoder", leftShooter.getEncoder().getVelocity());
    SmartDashboard.putNumber("Shooter/Right encoder", rightShooter.getEncoder().getVelocity());
    leftFaultAlert.setText("coral left:" + leftShooter.getFaults().toString()); leftFaultAlert.set(leftShooter.hasActiveFault());
    rightFaultAlert.setText("coral right:" + rightShooter.getFaults().toString()); rightFaultAlert.set(rightShooter.hasActiveFault());
    leftWarningAlert.setText("coral left:" + leftShooter.getWarnings().toString()); leftWarningAlert.set(leftShooter.hasActiveWarning());
    rightWarningAlert.setText("coral right:" + rightShooter.getWarnings().toString()); rightWarningAlert.set(rightShooter.hasActiveWarning());
  }
  public void set(double speed) {
    leftShooter.set(speed);
    rightShooter.set(speed);
  }
  public void stop() {
    leftShooter.stopMotor();
    rightShooter.stopMotor();
  }

  public double getLeftVelocity(){
    return leftShooter.getEncoder().getVelocity()/3.0;
  }
  public double getRightVelocity(){
    return leftShooter.getEncoder().getVelocity()/3.0;
  }
} 
