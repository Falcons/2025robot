// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.moveToTarget;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.driveTrain.SwerveSubsystem;

public class MoveToTag extends Command {

  SwerveSubsystem swerveSubsystem;
  private final Supplier<Boolean> fieldOriented;
  /** Creates a new MoveToTag. */
  public MoveToTag(SwerveSubsystem swerve, Supplier<Boolean> field) {
    this.swerveSubsystem = swerve;
    this.fieldOriented = field;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds chassisSpeeds;
    if (fieldOriented.get()) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed(), ySpeed(), turningSpeed(), swerveSubsystem.getRotation2d());
    } else {
      chassisSpeeds = new ChassisSpeeds(xSpeed(), ySpeed(), turningSpeed());
    }
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    
    swerveSubsystem.setModuleStates(moduleStates);
  }

  double turningSpeed() //think i changed it corectly from assisting driving to move to target
  {    
    // kP (constant of proportionality)=this is a hand-tuned number that determines the aggressiveness of our proportional control loop=if it is too high, the robot will oscillate around.=if it is too low, the robot will never reach its target=if the robot never turns in the correct direction, kP should be inverted.
    double kP = .035;
    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of=your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity =  LimelightHelpers.getTX("limelight-tag"); /* *kp */

    // convert to radians per second for our drive method
    //targetingAngularVelocity *= Drivetrain.kMaxAngularSpeed;
    targetingAngularVelocity *= DriveConstants.maxAngularSpeedRadiansPerSecond;
    //invert since tx is positive when the target is to the right of the crosshair
    //targetingAngularVelocity *= -1.0;
    if (targetingAngularVelocity < kP && targetingAngularVelocity > -kP) {
      targetingAngularVelocity = 0;
    }
    return targetingAngularVelocity;
  }
  double xSpeed(){
    double kP = .035;
    double X = LimelightHelpers.getTargetPose3d_RobotSpace("limelight-tag").getX();
    double poseX = swerveSubsystem.getPose().getX();
    double Xspeed = X - poseX;

    if (Xspeed < kP && Xspeed > -kP) {
      Xspeed = 0;
    }
    return Xspeed;
  }
  double ySpeed(){
    double kP = .035;
    double Y = LimelightHelpers.getTargetPose3d_RobotSpace("limelight-tag").getY();
    double poseY = swerveSubsystem.getPose().getY();
    double Yspeed = Y - poseY;

    if (Yspeed < kP && Yspeed > -kP) {
      Yspeed = 0;
    }
    return Yspeed;
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
