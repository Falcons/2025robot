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

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Coral extends SubsystemBase {

  private ShuffleboardTab tab = Shuffleboard.getTab("tab 3");
   private GenericEntry leftCoralSpeed =
      tab.add("Left Coral Speed", 0.03)
         .getEntry();
    private GenericEntry rightCoralSpeed =
    tab.add("Right Coral Speed", 0.2)
       .getEntry();

  private final SparkMax leftShooter, rightShooter;
  private SparkMaxConfig rightShooterConfig = new SparkMaxConfig();
  private SparkMaxConfig leftShooterConfig = new SparkMaxConfig();
  private Alert leftFaultAlert = new Alert("Faults", "", AlertType.kError);
  private Alert rightFaultAlert = new Alert("Faults", "", AlertType.kError);
  private Alert leftWarningAlert = new Alert("Warnings", "", AlertType.kWarning);
  private Alert rightWarningAlert = new Alert("Warnings", "", AlertType.kWarning);
  /** Creates a new coral thing. */
  public Coral() {
    this.leftShooter = new SparkMax(ShooterConstants.leftMotorCANID, MotorType.kBrushless);
    this.rightShooter = new SparkMax(ShooterConstants.rightMotorCANID, MotorType.kBrushless);
    rightShooterConfig.idleMode(IdleMode.kBrake);
    leftShooterConfig.idleMode(IdleMode.kBrake);
    leftShooterConfig.inverted(true);
    leftShooterConfig.smartCurrentLimit(20);
    leftShooter.configure(leftShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    rightShooter.configure(rightShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Coral/Left encoder", leftShooter.getEncoder().getVelocity());
    SmartDashboard.putNumber("Coral/Right encoder", rightShooter.getEncoder().getVelocity());
    leftFaultAlert.setText("coral left:" + leftShooter.getFaults().toString()); leftFaultAlert.set(leftShooter.hasActiveFault());
    rightFaultAlert.setText("coral right:" + rightShooter.getFaults().toString()); rightFaultAlert.set(rightShooter.hasActiveFault());
    leftWarningAlert.setText("coral left:" + leftShooter.getWarnings().toString()); leftWarningAlert.set(leftShooter.hasActiveWarning());
    rightWarningAlert.setText("coral right:" + rightShooter.getWarnings().toString()); rightWarningAlert.set(rightShooter.hasActiveWarning());
  }
  public void set(double left, double right) {
    SmartDashboard.putNumber("Coral/right speed", right);
    SmartDashboard.putNumber("Coral/left speed", left);
    leftShooter.set(leftCoralSpeed.getDouble(0.03));
    rightShooter.set(rightCoralSpeed.getDouble(0.2));
  }
  public void stop() {
    leftShooter.stopMotor();
    rightShooter.stopMotor();
  }//328 

  public double getLeftVelocity(){
    return leftShooter.getEncoder().getVelocity()/3.0;
  }
  public double getRightVelocity(){
    return leftShooter.getEncoder().getVelocity()/3.0;
  }
} 
