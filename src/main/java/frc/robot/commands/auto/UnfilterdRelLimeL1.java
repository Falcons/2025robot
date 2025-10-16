// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.coral.rawCoralSet;
import frc.robot.commands.elevator.ElevatorTrapezoidalMove;
import frc.robot.commands.moveToTarget.UnfilterdFollowTagG;
import frc.robot.subsystems.algae.Pivot;
import frc.robot.subsystems.driveTrain.SwerveSubsystem;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.shooter.Coral;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class UnfilterdRelLimeL1 extends SequentialCommandGroup {
    /** Creates a new UnfilteredRelLimeL1. */
  public UnfilterdRelLimeL1(SwerveSubsystem swerve, Elevator elevator, Pivot pivot, Coral coral) {
    // Add your commands in the addCommands() call, e.g.
    addCommands(
      new WaitCommand(3),
      new ElevatorTrapezoidalMove(elevator, ElevatorConstants.maxSpeed, ElevatorConstants.maxAcceleration, ElevatorConstants.coralL1).asProxy(),
      new UnfilterdFollowTagG(swerve),
      new Taxi(swerve, 0.5),
      new WaitCommand(1.0),
      new rawCoralSet(coral, ShooterConstants.L1LeftSpeed, ShooterConstants.L1RightSpeed).withTimeout(1).asProxy()
      // new PivotAndElevatorHome(pivot, elevator).asProxy()
     );
  }
}
