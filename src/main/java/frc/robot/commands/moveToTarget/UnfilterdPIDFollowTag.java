// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.moveToTarget;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.driveTrain.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class UnfilterdPIDFollowTag extends Command {
  SwerveSubsystem swerve;
  double[] targetPose, setpoints;
  /** Creates a new FollowTag. */
  public UnfilterdPIDFollowTag(SwerveSubsystem swerve) { 
    this.swerve = swerve;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setpoints = new double[] {0,0,0};
    try {
      if (LimelightHelpers.getTV("limelight-tag")) {
        targetPose = LimelightHelpers.getTargetPose_RobotSpace("limelight-tag");
      }else if (LimelightHelpers.getTV("limelight-end")) {
        targetPose = LimelightHelpers.getTargetPose_RobotSpace("limelight-end");
      }else targetPose = new double[]{0,0,0,0,0,0};
    } catch (Exception e) {
      System.err.println(e);
    }
    SmartDashboard.putNumber("Auto/Target Y", -targetPose[0]);
    SmartDashboard.putNumber("Auto/Target O", -targetPose[4]);
    SmartDashboard.putNumber("Auto/Target O Rad", Units.degreesToRadians(-targetPose[4]));
    setpoints[0] = swerve.getPose().getX() + -targetPose[2];
    setpoints[1] = swerve.getPose().getY() + -targetPose[0];
    setpoints[2] = swerve.getPose().getRotation().getRadians() + Units.degreesToRadians(-targetPose[4]);
    SmartDashboard.putNumber("Auto/real y", swerve.getPose().getY());
    SmartDashboard.putNumber("Auto/real O", swerve.getPose().getRotation().getRadians());
    SmartDashboard.putNumber("Auto/Setpoint Y", setpoints[1]);
    SmartDashboard.putNumber("Auto/Setpoint O", setpoints[2]);
    SmartDashboard.putNumber("Auto/Setpoint O Deg", Units.radiansToDegrees(setpoints[2]));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds chassisSpeeds;
    double Y = swerve.robotPIDCalc('y', swerve.getPose().getY(), setpoints[1]);
    Double O = swerve.robotPIDCalc('o', swerve.getPose().getRotation().getRadians(), setpoints[2]);
    chassisSpeeds = new ChassisSpeeds(0, Y, O);
    swerve.driveRobotRelative(chassisSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(targetPose[0])  <= 0.1 && Math.abs(targetPose[4]) <= 5;
  }
}