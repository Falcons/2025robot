// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.elevator.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TrapAndSmallPID extends SequentialCommandGroup {
  Elevator elevator;
  double endPos;
  /** Creates a new TrapAndSmallPID. */
  public TrapAndSmallPID(Elevator elevator, double maxV, double maxA, double endPos) {
    addRequirements(elevator);
    addCommands(
      new ElevatorTrapezoidalMove(elevator, maxV, maxA, endPos),
      new ElevatorSetSmallPid(elevator, endPos)
    );
  }
}
