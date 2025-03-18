// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.moveToTarget;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.driveTrain.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class UnfilterdFollowTagG extends SequentialCommandGroup {
  /** Creates a new FollowTagG. */
  public UnfilterdFollowTagG(SwerveSubsystem swerve, double[] offset) {
    addCommands(
      new UnfilterdFollowTag(swerve, offset),
      new UnfilterdFollowTagDrive(swerve, offset)
    );
  }
}
