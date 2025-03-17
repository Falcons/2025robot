// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorHoldVoltage extends Command {
  Elevator elevator;
  double high, mid, low;
  /** Creates a new ElevatorSetVoltage. */
  public ElevatorHoldVoltage(Elevator elevator, double high, double mid, double low) {
    this.elevator = elevator;
    this.high = high;
    this.mid = mid;
    this.low = low;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(this.getName() + " start");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double cVolts = mid;
    if (elevator.atDrop && !elevator.danger) cVolts = 0;
    elevator.setVoltage(cVolts);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println(this.getName() + " end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
