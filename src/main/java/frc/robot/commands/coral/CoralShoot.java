// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.coral;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.shooter.Coral;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralShoot extends Command {
  Supplier<Double> speed;
  Coral coral;
  Elevator elevator;
  /** Creates a new shoot. */
  public CoralShoot(Coral coral, Elevator elevator, Supplier<Double> speed) {
    this.coral = coral;
    this.elevator = elevator;
    this.speed = speed;
    addRequirements(coral);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double left = speed.get();
    double right = speed.get();
    // if(elevator.getEncoder() >= ElevatorConstants.coralL1-1 && elevator.getEncoder() <= ElevatorConstants.coralL1+0.5)left = 0;
    coral.set(left, right);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coral.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
