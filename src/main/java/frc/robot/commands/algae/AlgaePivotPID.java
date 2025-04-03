// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.algae;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algae.Pivot;
import frc.robot.subsystems.elevator.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaePivotPID extends Command {
  Pivot algae;
  Elevator elevator;
  double angle;
  /** Creates a new shoot. */
  public AlgaePivotPID(Pivot algae, Elevator elevator, double angle) {
    this.algae = algae;
    this.elevator = elevator;
    this.angle = angle;
    addRequirements(algae);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    algae.setPivotpid(angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    algae.stopPivot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return elevator.getEncoder() >= AlgaeConstants.MaxAlgaeHeight && angle < 160;
    return false;
  }
}
