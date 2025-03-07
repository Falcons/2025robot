// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.algae;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.subsystems.algae.Pivot;
import frc.robot.subsystems.elevator.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class pivotDefault extends Command {
  Pivot pivot;
  Elevator elevator;
  /** Creates a new pivotDefault. */
  public pivotDefault(Pivot pivot, Elevator elevator) {
    this.pivot = pivot;
    this.elevator =  elevator;
    addRequirements(pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(this.getName() + " start");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean atMin = elevator.getEncoder() < AlgaeConstants.MaxAlgaeHeight;
    SmartDashboard.putBoolean("elevator/at min algae", atMin);
      if (atMin && pivot.getAbsEncoderDeg() < 90){
        elevator.pause = true;
      //pivot.setPivotpid(AlgaeConstants.pivotMin);
    }else elevator.pause = false;
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
