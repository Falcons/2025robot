// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.algae;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.elevator.ElevatorTrapezoidalMove;
import frc.robot.subsystems.algae.Pivot;
import frc.robot.subsystems.elevator.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ElevatorAndPivotOut extends ParallelCommandGroup {
  /** Creates a new ElevatorAndPivotOut. */
  public ElevatorAndPivotOut(Pivot pivot, Elevator elevator, double setpoint) {
    addCommands(
      new ElevatorTrapezoidalMove(elevator, ElevatorConstants.maxSpeed, ElevatorConstants.maxAcceleration, setpoint).asProxy(),
      new PivotPid(pivot, AlgaeConstants.pivotOut)
    );
  }
}
