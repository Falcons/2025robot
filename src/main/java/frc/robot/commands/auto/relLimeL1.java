// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.coral.rawCoralSet;
import frc.robot.commands.elevator.ElevatorSetVoltage;
import frc.robot.commands.elevator.ElevatorTrapezoidalMove;
import frc.robot.commands.elevator.trapAndSmallPid;
import frc.robot.subsystems.driveTrain.SwerveSubsystem;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.shooter.Coral;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class relLimeL1 extends SequentialCommandGroup {
  /** Creates a new relLimeL1. */
  public relLimeL1(SwerveSubsystem swerve, Elevator elevator, Coral coral,double tagID) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
     new ElevatorTrapezoidalMove(elevator, ElevatorConstants.maxSpeed, ElevatorConstants.maxAcceleration, ElevatorConstants.coralL1).asProxy(),
     new FollowTagG(swerve, tagID),
     new wait(1.25),
     new rawCoralSet(coral, -0.10, -0.40).withTimeout(1)
     );
  }
}
