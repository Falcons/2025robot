// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.moveToTarget;

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
public class UnfilterdFollowTag extends Command {
  SwerveSubsystem swerve;
  List<Waypoint> waypoints;
  PathPlannerPath path;
  double[] targetPose;
  /** Creates a new FollowTag. */
  public UnfilterdFollowTag(SwerveSubsystem swerve) { 
    this.swerve = swerve;
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
    try {
      if (LimelightHelpers.getTV("limelight-tag")) {
        targetPose = LimelightHelpers.getTargetPose_RobotSpace("limelight-tag");
      }else if (LimelightHelpers.getTV("limelight-end")) {
        targetPose = LimelightHelpers.getTargetPose_RobotSpace("limelight-end");
      };
    } catch (Exception e) {
      System.err.println(e);
    }
    SmartDashboard.putNumber("auto/pose/rot/X", targetPose[0]);
    SmartDashboard.putNumber("auto/pose/rot/Y", targetPose[1]);
    SmartDashboard.putNumber("auto/pose/rot/Z", targetPose[2]);
    SmartDashboard.putNumber("auto/pose/rot/Yaw", targetPose[4]);
    chassisSpeeds = new ChassisSpeeds(0, -targetPose[0], Units.degreesToRadians(-targetPose[4]*0.85));
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
    if(Math.abs(targetPose[0])  <= 0.1){
      System.out.println("Finished");
      return true;
    }else{
      return false; // && Math.abs(targetPose[4]) <= 6;
    }
  }
}
