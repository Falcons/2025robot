// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.limelightConstants;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.driveTrain.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveToTag extends Command {
  SwerveSubsystem swerve;
  int tag;
  /** Creates a new MoveToTag. */
  public MoveToTag(SwerveSubsystem swerve, int tag) {
    this.swerve = swerve;
    this.tag = tag;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] tagPose;
    if (LimelightHelpers.getFiducialID("limelight-tag")==tag)tagPose = LimelightHelpers.getTargetPose_RobotSpace("limelight-tag");
    // ChassisSpeeds sp = new ChassisSpeeds(?tagPose[0])
    // swerve.driveRobotRelative();
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
