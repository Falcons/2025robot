// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.algae.IntakeForTime;
import frc.robot.commands.elevator.ElevatorMoveToEnd;
import frc.robot.subsystems.algae.Intake;
import frc.robot.subsystems.algae.Pivot;
import frc.robot.subsystems.elevator.Elevator;
public class TestCommandGroup extends SequentialCommandGroup {
  /** Creates a new TestCommandGroup. */
  public TestCommandGroup(Elevator elevator, Pivot algaeP, Intake algaeI) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ElevatorMoveToEnd(elevator, 0.2),
      new ElevatorMoveToEnd(elevator, -0.2),
      new IntakeForTime(algaeI, 0.2, 1),
      new IntakeForTime(algaeI, -0.2, 1)
    );
  }
}
