// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.coral.rawCoralSet;
import frc.robot.commands.elevator.ElevatorSetVoltage;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.shooter.Coral;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HoldAndShoot extends ParallelCommandGroup {
  /** Creates a new HoldAndShoot. */
  public HoldAndShoot(Elevator elevator, Coral coral) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ElevatorSetVoltage(elevator, 0.76).asProxy(),
      new rawCoralSet(coral, -0.10, -0.40)
    );
  }
}
