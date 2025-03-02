// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.algae;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algae.Pivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaePivotToggle extends Command {
  Pivot algae;
  boolean isPivotUp = true;
  /** Creates a new shoot. */
  public AlgaePivotToggle(Pivot algae) {
    this.algae = algae;
    addRequirements(algae);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isPivotUp = !isPivotUp;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // algae.togglePivot(isPivotUp);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    algae.stopPivot();
    isPivotUp = !isPivotUp;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
