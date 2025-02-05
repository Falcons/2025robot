// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

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

public class coral extends SubsystemBase {
  private final SparkMax leftShooter, rightShooter;
  private SparkMaxConfig shooterConfig;
  /** Creates a new coral thing. */
  public coral() {
    this.leftShooter = new SparkMax(AlgaeConstants.intakeMoterCANID, MotorType.kBrushless);
    this.rightShooter = new SparkMax(AlgaeConstants.intakeMoterCANID, MotorType.kBrushless);

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
  }
  public void set(double speed) {
    leftShooter.set(speed);
    rightShooter.set(speed);
  }
}
