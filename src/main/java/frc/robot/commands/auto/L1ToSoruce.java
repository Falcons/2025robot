// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.driveTrain.SwerveSubsystem;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.shooter.Coral;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class L1ToSoruce extends SequentialCommandGroup {
  /** Creates a new L1ToSoruce. */
  public L1ToSoruce(SwerveSubsystem swerve, Elevator elevator, Coral coral, double tagID) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new relLimeL1(swerve, elevator, coral, tagID),
      new relAutoDrive(swerve, new ChassisSpeeds(-1,0,0), 1),
      new relAutoDrive(swerve, new ChassisSpeeds(0,0,Units.degreesToRadians(-45)), 2),
      new relAutoDrive(swerve, new ChassisSpeeds(1,0,0), 0.5),
      new FollowTagG(swerve, 1)
    );
  }
}
