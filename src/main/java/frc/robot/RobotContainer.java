// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SwerveJoystick;
import frc.robot.commands.AllModulePID;
import frc.robot.commands.SwervePositionPIDTuning;
import frc.robot.commands.algaePivot;
import frc.robot.commands.shoot;
import frc.robot.commands.intake;
import frc.robot.commands.elevatorManual;
import frc.robot.subsystems.algae.algae;
import frc.robot.subsystems.shooter.coral;
import frc.robot.subsystems.driveTrain.SwerveSubsystem;
import frc.robot.subsystems.elevator.elevator;

public class RobotContainer {

  private final coral coral = new coral();
  private final algae algae = new algae();
  private final elevator elevator = new elevator();

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(0);

  private final SwerveSubsystem swerve = new SwerveSubsystem();

  SendableChooser<PathPlannerAuto> path_chooser = new SendableChooser<PathPlannerAuto>();
  public RobotContainer() {
    swerve.setDefaultCommand(new SwerveJoystick(
      swerve, 
      () -> -driver.getLeftY(), 
      () -> -driver.getLeftX(), 
      () -> -driver.getRightX(), 
      () -> !driver.getHID().getLeftBumper()));
    coral.setDefaultCommand(new shoot(coral, operator.getRightTriggerAxis())); // intake
    coral.setDefaultCommand(new shoot(coral, -operator.getLeftTriggerAxis())); //outake
    algae.setDefaultCommand(new algaePivot(algae, -operator.getLeftY())); // pivot
    elevator.setDefaultCommand(new elevatorManual(elevator, operator.getRightY())); // elevator
    configureBindings();

    SmartDashboard.putData("Reset Field Pose", new InstantCommand(() -> swerve.resetPose(new Pose2d())).ignoringDisable(true));
    path_chooser.setDefaultOption("none", null);
    path_chooser.addOption("figure 8", new PathPlannerAuto("better figure 8"));
    path_chooser.addOption("auto 1", new PathPlannerAuto("Auto 1"));
    SmartDashboard.putData(path_chooser);
  }

  private void configureBindings() {
    operator.x().whileTrue(new intake(algae, 0.1));
    operator.a().whileTrue(new intake(algae, -0.1));

    driver.povUpLeft().whileTrue(swerve.modulePIDTuning("Front Left"));
    driver.povUpRight().whileTrue(swerve.modulePIDTuning("Front Right"));
    driver.povDownLeft().whileTrue(swerve.modulePIDTuning("Back Left"));
    driver.povDownRight().whileTrue(swerve.modulePIDTuning("Back Right"));
  }

  public Command getAutonomousCommand() {
    return path_chooser.getSelected();
  }
}
