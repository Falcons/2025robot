// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.algae;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algae.Pivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaePivot extends Command {
  Supplier<Double> speed;
  Pivot algae;
  /** Creates a new shoot. */
  public AlgaePivot(Pivot algae, Supplier<Double> speed) {
    this.algae = algae;
    this.speed = speed;
    addRequirements(algae);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("algae pivot start");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    algae.setPivot(speed.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("algae pivot end");
    algae.stopPivot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
