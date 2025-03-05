// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.coral;

import java.net.Socket;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Airlock;
import frc.robot.subsystems.shooter.Coral;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralStep extends Command {
  Coral coral;
  Airlock airlock;
  Supplier<Double> speed;
  /** Creates a new CoralStep. */
  public CoralStep(Coral coral, Airlock airlock, Supplier<Double> speed) {
    this.coral = coral;
    this.airlock = airlock;
    this.speed = speed;
    addRequirements(coral);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(this.getName() + " start");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = this.speed.get();
    if(!airlock.checkStep()) speed = 0;
    coral.set(speed, speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println(this.getName() + " ends");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
