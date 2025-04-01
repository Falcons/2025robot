// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.algae.AlgaeIntake;
import frc.robot.commands.elevator.ElevatorTrapezoidalMove;
import frc.robot.subsystems.algae.Intake;
import frc.robot.subsystems.elevator.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ElevatorAndAlgae extends ParallelRaceGroup {
  /** Creates a new ElevatorAndAlgae. */
  public ElevatorAndAlgae(Elevator elevator, Intake intake) {
    addCommands(
      new ElevatorTrapezoidalMove(elevator, ElevatorConstants.maxSpeed, ElevatorConstants.maxAcceleration, ElevatorConstants.coralL3),
      new AlgaeIntake(intake, -1.0)
    );
  }
}
