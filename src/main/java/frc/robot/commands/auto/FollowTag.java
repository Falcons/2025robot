// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.driveTrain.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FollowTag extends Command {
  SwerveSubsystem swerve;
  double tagID;
  List<Waypoint> waypoints;
  PathPlannerPath path;
  double[] targetPose, offset;
  /** Creates a new FollowTag. */
  public FollowTag(SwerveSubsystem swerve, double tagID, double[] offset) { 
    this.swerve = swerve;
    this.tagID = tagID;
    this.offset = offset;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds chassisSpeeds;
    try {
      if (LimelightHelpers.getFiducialID("limelight-tag") == tagID) {
        System.out.println("using limeligt-tag");
        targetPose = LimelightHelpers.getTargetPose_RobotSpace("limelight-tag");
      }else if (LimelightHelpers.getFiducialID("limelight-end") == tagID) {
        System.out.println("using limeligt-end");
        targetPose = LimelightHelpers.getTargetPose_RobotSpace("limelight-end");
      }else targetPose = new double[]{0,0,0,0,0,0};
    } catch (Exception e) {
      System.err.println(e);
    }
    SmartDashboard.putNumberArray("Auto/target pose", targetPose);
    chassisSpeeds = new ChassisSpeeds(0, -targetPose[0]+offset[0], Units.degreesToRadians(-targetPose[4]));
    swerve.driveRobotRelative(chassisSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(targetPose[0]+offset[0])  <= 0.1 && Math.abs(targetPose[4]) <= 5;
  }
}
