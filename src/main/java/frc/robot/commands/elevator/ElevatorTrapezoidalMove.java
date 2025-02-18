// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorTrapezoidalMove extends Command {
  Elevator elevator;
  double endPos;
  TrapezoidProfile.Constraints constraints;
  TrapezoidProfile profile;
  TrapezoidProfile.State current, end;
  /** Creates a new ElevatorTrapezoidalMove. 
   * @param elevator elevator subsystem
   * @param maxV max velocity
   * @param maxA max acceleration
   * @param endPos end position
  */
  public ElevatorTrapezoidalMove(Elevator elevator, double maxV, double maxA, double endPos) {
    this.elevator = elevator;
    this.endPos = endPos;
    constraints = new TrapezoidProfile.Constraints(maxV, maxA);
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    profile = new TrapezoidProfile(constraints);
    current = new TrapezoidProfile.State(elevator.getEncoder(), elevator.getVelocity());
    end = new TrapezoidProfile.State(endPos, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    current = profile.calculate(0.2, current, end);
    elevator.setPID(current.position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return profile.timeLeftUntil(endPos) <= 0;
  }
}
