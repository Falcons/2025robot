// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.elevator.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class setElevatorTargetPos extends Command {
  Elevator elevator;
  Supplier<Double> adjustment;
  Alert maxAlert = new Alert("elevator target at max", AlertType.kWarning);
  Alert minAlert = new Alert("elevator target at min", AlertType.kWarning);
  /** Creates a new setElevatorTargetPos. */
  public setElevatorTargetPos(Elevator elevator, Supplier<Double> adjustment) {
    this.elevator = elevator;
    this.adjustment = adjustment;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(this.getName() + " start");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double adjust = adjustment.get();
    boolean isMax = (adjust + elevator.targetPos >= ElevatorConstants.encoderMax); maxAlert.set(isMax); if(isMax) adjust = 0;
    boolean isMin = (adjust + elevator.targetPos <= ElevatorConstants.encoderMin); minAlert.set(isMin); if(isMin) adjust = 0; 
    elevator.targetPos += adjust;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println(this.getName() + " end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
