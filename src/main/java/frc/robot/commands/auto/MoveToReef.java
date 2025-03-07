// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.ModuleConstants;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.driveTrain.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveToReef extends Command {
  SwerveSubsystem swerve;
  boolean team;
  List<Waypoint> waypoints;
  LimelightTarget_Fiducial tag;
  /** Creates a new MoveToReef. */
  public MoveToReef(SwerveSubsystem swerve, boolean team) {

    this.swerve = swerve;
    this.team = team;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d robotRel = swerve.getPose().relativeTo(tag.getTargetPose_RobotSpace2D());
    double[] poserel = {robotRel.getX(), robotRel.getY(), robotRel.getRotation().getDegrees()};
    SmartDashboard.putNumberArray("reletive", poserel);
     // Create a list of waypoints from poses. Each pose represents one waypoint.
    // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
    waypoints = PathPlannerPath.waypointsFromPoses(
        // new Pose2d(swerve.getPose().getX(), swerve.getPose().getY(), new Rotation2d(swerve.getChassisSpeeds().omegaRadiansPerSecond)),
        );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   

  PathConstraints constraints = new PathConstraints(ModuleConstants.driveMaxSpeedMPS, 1.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
  // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage

  // Create the path using the waypoints created above
  PathPlannerPath path = new PathPlannerPath(
      waypoints,
      constraints,
      null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
      new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
  );

  // Prevent the path from being flipped if the coordinates are already correct
  path.preventFlipping = true;
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
