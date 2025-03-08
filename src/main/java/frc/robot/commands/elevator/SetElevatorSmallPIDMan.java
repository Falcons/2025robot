// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetElevatorSmallPIDMan extends Command {
  Elevator elevator;
  Double offset;
  double target;
  /** Creates a new SetElevatorPID. */
  public SetElevatorSmallPIDMan(Elevator elevator, Double offset) {
    this.elevator = elevator;
    this.offset = offset;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(this.getName() + " start");
    target = elevator.getEncoder()+offset;
    SmartDashboard.putNumber("Elevator/PID/Small/target", target);    
    SmartDashboard.putNumber("Elevator/PID/Small/offset", offset);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.setSmallPID(target);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println(this.getName() + " end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return elevator.atSetpoint();
    return false; //set to true if hold bad
  }
}
