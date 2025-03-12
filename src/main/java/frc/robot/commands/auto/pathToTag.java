// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.lang.annotation.Target;
import java.util.List;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ModuleConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.driveTrain.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class pathToTag extends Command {
  SwerveSubsystem swerve;
  double tagID;
  List<Waypoint> waypoints;
  PathPlannerPath path;
  /** Creates a new pathToTag. */
  public pathToTag(SwerveSubsystem swerve, double tagID) { 
    this.swerve = swerve;
    this.tagID = tagID;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   Pose2d start = new Pose2d(swerve.getPose().getX(), swerve.getPose().getY(), Rotation2d.fromRadians(swerve.getChassisSpeeds().omegaRadiansPerSecond));
   Pose2d target = start;
   if(LimelightHelpers.getTV("limelight-tag")){
    if(LimelightHelpers.getFiducialID("limelight-tag") == tagID){
       double[] poseA = LimelightHelpers.getTargetPose_RobotSpace("limelight-tag");
       target.plus(new Transform2d(poseA[0], poseA[1], Rotation2d.fromDegrees(poseA[4])));
      }
    }
    SmartDashboard.putNumber("auto/start x", start.getX());
    SmartDashboard.putNumber("auto/start y", start.getY());
    SmartDashboard.putNumber("auto/start rot", start.getRotation().getDegrees());
    SmartDashboard.putNumber("auto/target x", start.getX());
    SmartDashboard.putNumber("auto/target y", start.getY());
    SmartDashboard.putNumber("auto/targrt rot", start.getRotation().getDegrees());
    waypoints = PathPlannerPath.waypointsFromPoses(
      start,
      target
    );

    
  PathConstraints constraints = new PathConstraints(ModuleConstants.driveMaxSpeedMPS, 1.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
  // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage

  // Create the path using the waypoints created above
  path = new PathPlannerPath(
      waypoints,
      constraints,
      null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
      new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
  );

  // Prevent the path from being flipped if the coordinates are already correct
  path.preventFlipping = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve.followPathCommand(path);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
