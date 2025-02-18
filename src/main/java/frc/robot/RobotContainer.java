// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.commands.algae.AlgaePivot;
import frc.robot.commands.algae.AlgaePivotFeedforward;
import frc.robot.commands.algae.Intake;
import frc.robot.commands.coral.CoralShoot;
import frc.robot.commands.driveTrain.SwerveJoystick;
// import frc.robot.commands.driveTrain.SwerveToggleSlowMode; //keep this for debug -madness
// import frc.robot.commands.elevator.ElevatorManual;
import frc.robot.commands.elevator.ElevatorToggleSlowMode;
import frc.robot.commands.elevator.ElevatorTrapezoidalMove;
import frc.robot.subsystems.Airlock;
import frc.robot.subsystems.algae.Algae;
import frc.robot.subsystems.shooter.Coral;
import frc.robot.subsystems.driveTrain.SwerveSubsystem;
import frc.robot.subsystems.elevator.Elevator;
public class RobotContainer {

  private final Airlock airlock = new Airlock();
  private final Elevator elevator = new Elevator(airlock);
  private final Coral coral = new Coral(airlock);
  private final Algae algae = new Algae();
  private Map<String, Command> commandList = new HashMap<>();

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(0);

  private final SwerveSubsystem swerve = new SwerveSubsystem();

  SendableChooser<Command> path_chooser = new SendableChooser<Command>();
  public RobotContainer() {
    swerve.setDefaultCommand(new SwerveJoystick(
      swerve, 
      () -> -driver.getLeftY(), 
      () -> -driver.getLeftX(), 
      () -> -driver.getRightX(), 
      () -> !driver.getHID().getLeftBumper()));
    coral.setDefaultCommand(new CoralShoot(coral, operator.getRightTriggerAxis())); // outake
    // algae.setDefaultCommand(new AlgaePivot(algae, operator.getLeftY())); // pivot
    algae.setDefaultCommand(new AlgaePivotFeedforward(algae, algae.getPivotPos()+operator.getLeftY(), operator.getLeftY(), 1)); //idk im quessing for this -madness
    // elevator.setDefaultCommand(new ElevatorManual(elevator, operator.getRightY())); // elevator
    elevator.setDefaultCommand(new ElevatorTrapezoidalMove(elevator, 10, 1, elevator.getEncoder()+operator.getRightY())); //idk im quessing for this -madness
    configureBindings();

    commandList.put("intake algae", new Intake(algae, 1));
    commandList.put("outTake algae", new Intake(algae, -1));
    commandList.put("outTake coral", new CoralShoot(coral, 1));
    commandList.put("set elevator bottom", new ElevatorTrapezoidalMove(elevator,10,1, 0));
    commandList.put("set elevator L1", new ElevatorTrapezoidalMove(elevator,10,1, 1));
    commandList.put("set elevator L2", new ElevatorTrapezoidalMove(elevator,10,1, 2));
    commandList.put("set elevator L3", new ElevatorTrapezoidalMove(elevator,10,1, 3));
    commandList.put("set elevator L4", new ElevatorTrapezoidalMove(elevator,10,1, 4));

    SmartDashboard.putData("Reset Field Pose", new InstantCommand(() -> swerve.resetPose(new Pose2d())).ignoringDisable(true));
    path_chooser = AutoBuilder.buildAutoChooser("default");
    SmartDashboard.putData("auto", path_chooser);
    NamedCommands.registerCommands(commandList);
  }

  private void configureBindings() {
    operator.x().whileTrue(new Intake(algae, 0.1)); // intake algae
    operator.a().whileTrue(new Intake(algae, -0.1)); // shoot algae
    operator.y().onTrue(new ElevatorToggleSlowMode(elevator));

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