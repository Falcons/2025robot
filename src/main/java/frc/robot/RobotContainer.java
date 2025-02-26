// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.HashMap;
import java.util.Map;
import java.util.jar.Attributes.Name;

import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.algae.AlgaePivot;
import frc.robot.commands.algae.AlgaePivotFeedforward;
import frc.robot.commands.algae.IntakeForTime;
import frc.robot.commands.algae.AlgaeIntake;
import frc.robot.commands.coral.CoralShoot;
import frc.robot.commands.driveTrain.SwerveJoystick;
import frc.robot.commands.driveTrain.SwerveToggleSlowMode;
import frc.robot.commands.elevator.ElevatorManual;
import frc.robot.commands.elevator.ElevatorToggleSlowMode;
import frc.robot.commands.elevator.ElevatorTrapezoidalMove;
import frc.robot.subsystems.Airlock;
import frc.robot.subsystems.algae.Pivot;
import frc.robot.subsystems.shooter.Coral;
import frc.robot.subsystems.driveTrain.SwerveSubsystem;
import frc.robot.subsystems.elevator.Elevator;
public class RobotContainer {

  private final Airlock airlock = new Airlock();
  private final Elevator elevator = new Elevator(airlock);
  private final Coral coral = new Coral(airlock);
  private final Pivot algaeP = new Pivot();
  private final frc.robot.subsystems.algae.Intake algaeI = new frc.robot.subsystems.algae.Intake();
  private final SwerveSubsystem swerve = new SwerveSubsystem();
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(0);

  private final double globalSpeedMod = 0.1;
  SendableChooser<Command> path_chooser = new SendableChooser<Command>();
  public RobotContainer() {
    swerve.setDefaultCommand(new SwerveJoystick(
      swerve, 
      () -> -driver.getLeftY(), 
      () -> -driver.getLeftX(), 
      () -> -driver.getRightX(), 
      () -> !driver.getHID().getLeftBumper()));
    coral.setDefaultCommand(new CoralShoot(coral, operator.getRightTriggerAxis())); // outake
    algaeP.setDefaultCommand(new AlgaePivot(algaeP, operator.getLeftY())); // pivot
    // algaeP.setDefaultCommand(new AlgaePivotFeedforward(algaeP, algaeP.getPivotPos()+operator.getLeftY(), 1*globalSpeedMod)); //idk im quessing for this -madness
    elevator.setDefaultCommand(new ElevatorManual(elevator, operator.getRightY())); // elevator

    configureBindings();

    NamedCommands.registerCommand("intake algae", new AlgaeIntake(algaeI, 1*globalSpeedMod));
    NamedCommands.registerCommand("outTake algae", new IntakeForTime(algaeI, -1*globalSpeedMod, 0.5));
    NamedCommands.registerCommand("outTake coral", new CoralShoot(coral, 1*globalSpeedMod));
    NamedCommands.registerCommand("set elevator bottom", new ElevatorTrapezoidalMove(elevator, ElevatorConstants.maxSpeed*globalSpeedMod, ElevatorConstants.maxAcceleration, ElevatorConstants.TOFTriggerBottom[0]));
    NamedCommands.registerCommand("set elevator L1", new ElevatorTrapezoidalMove(elevator,ElevatorConstants.maxSpeed*globalSpeedMod,ElevatorConstants.maxAcceleration, ElevatorConstants.TOFTriggerL1[0]));
    NamedCommands.registerCommand("set elevator L2", new ElevatorTrapezoidalMove(elevator,ElevatorConstants.maxSpeed*globalSpeedMod,ElevatorConstants.maxAcceleration, ElevatorConstants.TOFTriggerL2[0]));
    NamedCommands.registerCommand("set elevator L3", new ElevatorTrapezoidalMove(elevator,ElevatorConstants.maxSpeed*globalSpeedMod,ElevatorConstants.maxAcceleration, ElevatorConstants.TOFTriggerL3[0]));
    NamedCommands.registerCommand("set elevator L4", new ElevatorTrapezoidalMove(elevator,ElevatorConstants.maxSpeed*globalSpeedMod,ElevatorConstants.maxAcceleration, ElevatorConstants.TOFTriggerL4[0])); 

    SmartDashboard.putData("Reset Field Pose", new InstantCommand(() -> swerve.resetPose(new Pose2d())).ignoringDisable(true));
    path_chooser = AutoBuilder.buildAutoChooserWithOptionsModifier("default", 
		stream -> 
		stream.filter(auto -> !auto.getName().startsWith("."))
);
    SmartDashboard.putData("auto", path_chooser);
  }
  
  private void configureBindings() {
    operator.x().whileTrue(new AlgaeIntake(algaeI, 1*globalSpeedMod)); // intake algae
    operator.a().whileTrue(new AlgaeIntake(algaeI, -1*globalSpeedMod)); // shoot algae
    operator.y().onTrue(new ElevatorToggleSlowMode(elevator));

    operator.povDown().onTrue(new ElevatorTrapezoidalMove(elevator,10*globalSpeedMod,1, 1));
    operator.povLeft().onTrue(new ElevatorTrapezoidalMove(elevator,10*globalSpeedMod,1, 2));
    operator.povUp().onTrue(new ElevatorTrapezoidalMove(elevator,10*globalSpeedMod,1, 3));
    operator.povRight().onTrue(new ElevatorTrapezoidalMove(elevator,10*globalSpeedMod,1, 4));
    //driver.y().onTrue(new SwerveToggleSlowMode(swerve)); made automatic | only use in dubug -madness
    driver.povUpLeft().whileTrue(swerve.modulePIDTuning("Front Left"));
    driver.povUpRight().whileTrue(swerve.modulePIDTuning("Front Right"));
    driver.povDownLeft().whileTrue(swerve.modulePIDTuning("Back Left"));
    driver.povDownRight().whileTrue(swerve.modulePIDTuning("Back Right"));
    //swerve.resetPose(null);
  }

  public Command getAutonomousCommand() {
    return path_chooser.getSelected();
  }
  public Command getTestCommand() {
    return null;
  }
}