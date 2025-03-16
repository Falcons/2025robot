// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.coral.RawShootForTime;
import frc.robot.commands.elevator.ElevatorTrapezoidalMove;
import frc.robot.subsystems.driveTrain.SwerveSubsystem;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.shooter.Coral;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DRL1 extends SequentialCommandGroup {
  /** Creates a new DRL1. */
  public DRL1(SwerveSubsystem swerve, Elevator elevator, Coral coral) {
    addCommands(
      new Taxi(swerve, 2.5),
      new ElevatorTrapezoidalMove(elevator, ElevatorConstants.maxSpeed, ElevatorConstants.maxAcceleration, ElevatorConstants.coralL1+0.5),
      new RawShootForTime(coral, 0, -20, 2)
    );
  }
}
