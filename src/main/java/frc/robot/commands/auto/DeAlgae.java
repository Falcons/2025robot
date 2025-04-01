// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.algae.pivotPidToggle;
import frc.robot.commands.elevator.ElevatorTrapezoidalMove;
import frc.robot.commands.moveToTarget.UnfilterdFollowTagG;
import frc.robot.subsystems.algae.Intake;
import frc.robot.subsystems.algae.Pivot;
import frc.robot.subsystems.driveTrain.SwerveSubsystem;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.shooter.Coral;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DeAlgae extends SequentialCommandGroup {
    double[] offset = {1,0,0};
    /** Creates a new relLimeL1. */
  public DeAlgae(SwerveSubsystem swerve, Elevator elevator, Coral coral, Pivot pivot, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    addCommands(
      new UnfilterdFollowTagG(swerve, offset),
      new ElevatorTrapezoidalMove(elevator, ElevatorConstants.maxSpeed, ElevatorConstants.maxAcceleration, ElevatorConstants.algaeL3).asProxy(),
      new pivotPidToggle(pivot)
     );
  }
}
