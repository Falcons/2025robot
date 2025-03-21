// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driveTrain;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.driveTrain.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AllModulePID extends ParallelCommandGroup {
  private final SwerveSubsystem swerveSubsystem;
  public AllModulePID(SwerveSubsystem swerve) {
    this.swerveSubsystem = swerve;
    addRequirements(swerveSubsystem);
    addCommands(
    swerveSubsystem.modulePIDTuning("Front Left"),
    swerveSubsystem.modulePIDTuning("Front Right"),
    swerveSubsystem.modulePIDTuning("Back Left"),
    swerveSubsystem.modulePIDTuning("Back Right"),
    new InstantCommand(swerveSubsystem::allModuleSetpoint)
    );
  }
}
