// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.algae;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.subsystems.algae.Pivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class pivotPidToggle extends Command {
  Pivot pivot;
  boolean out;
  /** Creates a new pivotPidToggle. */
  public pivotPidToggle(Pivot pivot) {
    this.pivot = pivot;
    addRequirements(pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    out = pivot.getAbsEncoderDeg() < AlgaeConstants.pivotMax-1;
    if(out) pivot.setPivotpid(Units.degreesToRadians(AlgaeConstants.pivotMax));
    else pivot.setPivotpid(Units.degreesToRadians(AlgaeConstants.pivotOut));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
