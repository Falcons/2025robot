// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.driveTrain.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PIDtest extends Command {
  SwerveSubsystem swerve;
  ChassisSpeeds speeds;
  double setpoint;
  PIDController pid = new PIDController(0.1, 0, 0);
  /** Creates a new Taxi. */
  public PIDtest(SwerveSubsystem swerve, double setpoint) {
    this.swerve = swerve;
    this.setpoint = setpoint;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    speeds = new ChassisSpeeds(0, swerve.robotPIDCalc('y', swerve.getPose().getY(), setpoint), 0);//swerve.robotPIDCalc('o', swerve.getPose().getRotation().getRadians(),Units.degreesToRadians(setpoint)));
    swerve.driveRobotRelative(speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
