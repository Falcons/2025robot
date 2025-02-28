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
  private final SparkMax intakeMotor;
  private SparkMaxConfig intakeConfig;

  Alert intakeFaultAlert = new Alert("Faults", "", AlertType.kError);
  Alert intakeWarningAlert = new Alert("Warnings", "", AlertType.kWarning);
  double previousCurrent = 0;
  /** Creates a new intake. */
  public Intake() {
    this.intakeMotor = new SparkMax(AlgaeConstants.intakeMotorCANID, MotorType.kBrushless);
    intakeConfig = new SparkMaxConfig();
    intakeConfig.idleMode(IdleMode.kBrake);

    intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }
  public void stopIntake() {
    intakeMotor.stopMotor();
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake/Encoder", intakeMotor.getEncoder().getPosition());
    intakeFaultAlert.setText("algea intake:" + intakeMotor.getFaults().toString()); intakeFaultAlert.set(intakeMotor.hasActiveFault());
    intakeWarningAlert.setText("algea intake:" + intakeMotor.getFaults().toString()); intakeWarningAlert.set(intakeMotor.hasActiveWarning());
  }
  public void setIntake(double speed) {
    intakeMotor.set(speed);
  }
  public double getIntakeVel() {
    return intakeMotor.getEncoder().getVelocity();
  }
  public double getIntakeCurrent() {
    return intakeMotor.getOutputCurrent();
  }
  public boolean intakeCurrentSpike(){
    if (previousCurrent - getIntakeCurrent() > AlgaeConstants.voltageSpikeDifference) {
      return true;
    }
    previousCurrent = getIntakeCurrent();
    return false;
  }
}
