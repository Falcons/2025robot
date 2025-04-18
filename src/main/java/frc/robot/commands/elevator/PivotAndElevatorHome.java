// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.algae.PivotPid;
import frc.robot.subsystems.algae.Pivot;
import frc.robot.subsystems.elevator.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PivotAndElevatorHome extends ParallelCommandGroup {
  /** Creates a new PivotAndElevatorHome. */
  public PivotAndElevatorHome(Pivot pivot, Elevator elevator) {
    addCommands(
      new PivotPid(pivot, AlgaeConstants.pivotMax),
      new ElevatorTrapezoidalMove(elevator, ElevatorConstants.maxSpeed, ElevatorConstants.maxAcceleration, ElevatorConstants.Min).asProxy()
    );
  }
}
