// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.moveToTarget;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.driveTrain.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PIDOffset extends Command {
  SwerveSubsystem swerve;
  double[] setpoints;
  boolean rotate;
  /** Creates a new FollowTag. */
  public PIDOffset(SwerveSubsystem swerve, double[] setpoints, boolean rotate) { 
    this.swerve = swerve;
    this.setpoints = setpoints;
    this.setpoints[2] = Units.degreesToRadians(setpoints[2]);
    this.rotate = rotate;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(this.getName() + " start");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds chassisSpeeds;
    double X = swerve.robotPIDCalc('x', swerve.getPose().getX(), setpoints[0] + swerve.getPose().getX());
    double Y = swerve.robotPIDCalc('y', swerve.getPose().getY(), setpoints[1] + swerve.getPose().getY());
    double O = 0;
    if (rotate){
      O = swerve.robotPIDCalc('o', swerve.getPose().getRotation().getRadians(), setpoints[2]);
    }
    chassisSpeeds = new ChassisSpeeds(X, Y, O);
    swerve.driveRobotRelative(chassisSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println(this.getName() + " end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return Math.abs(targetPose[0])  <= 0.1 && Math.abs(targetPose[4]) <= 5;
    return false;
  }
}