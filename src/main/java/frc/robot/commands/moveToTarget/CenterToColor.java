// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.moveToTarget;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.driveTrain.SwerveSubsystem;

public class CenterToColor extends Command {

  SwerveSubsystem swerveSubsystem;
  
  public CenterToColor(SwerveSubsystem swerve, Supplier<Boolean> field) {
    this.swerveSubsystem = swerve;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds chassisSpeeds;

    chassisSpeeds = new ChassisSpeeds(Speed(), Speed(), 0);

    swerveSubsystem.driveRobotRelative(chassisSpeeds);
  }
  double Speed(){
    double kP = .035;
    double Xspeed = LimelightHelpers.getTX("limelight-colour");

    if (Xspeed < kP && Xspeed > -kP) {
      Xspeed = 0;
    }
    return Xspeed;
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
