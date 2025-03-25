// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.coral.CoralShoot;
import frc.robot.commands.coral.RawShootForTime;
import frc.robot.commands.elevator.ElevatorTrapezoidalMove;
import frc.robot.commands.moveToTarget.UnfilterdFollowTagG;
import frc.robot.subsystems.algae.Pivot;
import frc.robot.subsystems.driveTrain.SwerveSubsystem;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.shooter.Coral;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class UnfilterdRelLimeL2 extends SequentialCommandGroup {
    double[] offset = {0,0,0};
    /** Creates a new UnfilteredRelLimeL1. */
  public UnfilterdRelLimeL2(SwerveSubsystem swerve, Elevator elevator, Pivot pivot, Coral coral, double lOrR) {
    // Add your commands in the addCommands() call, e.g.
    addCommands( 
      new Taxi(swerve, 1.0),
      new UnfilterdFollowTagG(swerve, offset),
      new relAutoDrive(swerve, new ChassisSpeeds(0.25,lOrR, 0), 0.4),
      new ElevatorTrapezoidalMove(elevator, ElevatorConstants.maxSpeed, ElevatorConstants.maxAcceleration, ElevatorConstants.coralL2).asProxy(),
      // new CoralShoot(coral, elevator, () -> -0.30)
      new RawShootForTime(coral, -0.30, -0.30, 2).asProxy()
     );
  }
}
