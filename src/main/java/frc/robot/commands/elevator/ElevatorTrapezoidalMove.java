// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.elevator.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorTrapezoidalMove extends Command {
  Elevator elevator;
  double endPos;
  TrapezoidProfile.Constraints constraints;
  TrapezoidProfile profile;
  TrapezoidProfile.State current, end, pos;
  Timer timer = new Timer();
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
    System.out.println(this.getName() + " start");
    SmartDashboard.putNumber("trap/end", endPos);
    profile = new TrapezoidProfile(constraints);
    current = new TrapezoidProfile.State(elevator.getEncoder(), elevator.getVelocity()/60.0);
    end = new TrapezoidProfile.State(endPos, 0);
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {//TODO: fix real state not matching target
    current = profile.calculate(0.05, current, end);
    double pos = current.position;
    if(current.position < ElevatorConstants.Min) {System.err.println("position under range"); pos = ElevatorConstants.Min;}
    if(current.position > ElevatorConstants.Max) {System.err.println("position above range"); pos = ElevatorConstants.Max;}
    elevator.setPID(pos);
    SmartDashboard.putNumber("trap/timer", timer.get());
    SmartDashboard.putNumber("trap/pos", pos);
    SmartDashboard.putNumber("trap/target vol", current.velocity);
    SmartDashboard.putNumber("trap/real vol", elevator.getVelocity()/60.0);
    SmartDashboard.putNumber("trap/target pos", current.position);
    SmartDashboard.putNumber("trap/real pos", elevator.getEncoder());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println(this.getName() + " end");
    elevator.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (elevator.getEncoder() >= endPos-0.15 && elevator.getEncoder() <= endPos+0.15) || (DriverStation.isAutonomous() && profile.isFinished(timer.get()-2.5));
  }
}
