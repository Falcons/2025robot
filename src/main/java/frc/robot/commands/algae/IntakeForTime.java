// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.algae;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algae.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeForTime extends Command {
  Timer timer = new Timer();
  Intake pivot;
  double speed;
  double time;
  /** Creates a new IntakeForTime. 
   * @param time time in seconds
  */
  public IntakeForTime(Intake pivot, double speed, double time) {
    this.pivot = pivot;
    this.speed = speed;
    this.time = time;
    addRequirements(pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivot.setIntake(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivot.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() <= time;
  }
}
