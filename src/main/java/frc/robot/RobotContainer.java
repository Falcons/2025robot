// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.file.Path;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import au.grapplerobotics.CanBridge;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.algae.AlgaePivot;
import frc.robot.commands.algae.IntakeForTime;
import frc.robot.commands.algae.PivotPid;
import frc.robot.commands.algae.intakeVoltage;
import frc.robot.commands.auto.MoveToReef;
import frc.robot.commands.algae.pivotDefault;
import frc.robot.commands.algae.AlgaeIntake;
import frc.robot.commands.coral.CoralShoot;
import frc.robot.commands.coral.CoralStep;
import frc.robot.commands.coral.rawCoralSet;
import frc.robot.commands.driveTrain.SwerveJoystick;
import frc.robot.commands.elevator.ElevatorManual;
import frc.robot.commands.elevator.ElevatorSetVoltage;
import frc.robot.commands.elevator.ElevatorTrapezoidalMove;
import frc.robot.subsystems.Airlock;
import frc.robot.subsystems.algae.Pivot;
import frc.robot.subsystems.shooter.Coral;
import frc.robot.subsystems.driveTrain.SwerveSubsystem;
import frc.robot.subsystems.elevator.Elevator;
public class RobotContainer {

  private final Airlock airlock = new Airlock();
  private final Elevator elevator = new Elevator(airlock);
  private final Coral coral = new Coral();
  private final Pivot algaeP = new Pivot();
  private final frc.robot.subsystems.algae.Intake algaeI = new frc.robot.subsystems.algae.Intake();
  private final SwerveSubsystem swerve = new SwerveSubsystem();
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);


  private final double globalSpeedMod = 0.2;
  private final double operatorRSDeadZone = 0.1;
  private final double operatorRTDeadZone = 0.01;
  // SendableChooser<Command> path_chooser = new SendableChooser<Command>();
  SendableChooser<Command> auto_chooser = new SendableChooser<Command>();
  public RobotContainer() { 
    CanBridge.runTCP();
    swerve.setDefaultCommand(new SwerveJoystick(
      swerve, 
      () -> -driver.getLeftY()*globalSpeedMod, 
      () -> -driver.getLeftX()*globalSpeedMod, 
      () -> -driver.getRightX()*globalSpeedMod, 
      () -> !driver.getHID().getLeftBumper()));
      coral.setDefaultCommand(new CoralStep(coral, airlock, () -> -0.1));
      // algaeP.setDefaultCommand(new pivotDefault(algaeP, elevator));
      algaeP.setDefaultCommand(new AlgaePivot(algaeP, () -> operator.getLeftY()*globalSpeedMod)); // pivot
      // algaeI.setDefaultCommand(new intakeVoltage(algaeI, () -> 5.0));
    elevator.setDefaultCommand(new ElevatorSetVoltage(elevator, 0.75));

    configureBindings();

    NamedCommands.registerCommand("intake algae", new AlgaeIntake(algaeI, 1*globalSpeedMod));
    NamedCommands.registerCommand("outTake algae", new IntakeForTime(algaeI, -1*globalSpeedMod, 0.5));
    NamedCommands.registerCommand("outTake coral", new CoralShoot(coral, elevator,() -> 1.0));
    NamedCommands.registerCommand("set elevator bottom", new ElevatorTrapezoidalMove(elevator, ElevatorConstants.maxSpeed, ElevatorConstants.maxAcceleration, ElevatorConstants.Min));
    NamedCommands.registerCommand("set elevator L1", new ElevatorTrapezoidalMove(elevator,ElevatorConstants.maxSpeed,ElevatorConstants.maxAcceleration, ElevatorConstants.coralL1));
    NamedCommands.registerCommand("set elevator L2", new ElevatorTrapezoidalMove(elevator,ElevatorConstants.maxSpeed,ElevatorConstants.maxAcceleration, ElevatorConstants.coralL2));
    NamedCommands.registerCommand("set elevator L3", new ElevatorTrapezoidalMove(elevator,ElevatorConstants.maxSpeed,ElevatorConstants.maxAcceleration, ElevatorConstants.coralL3));
    NamedCommands.registerCommand("set elevator L4", new ElevatorTrapezoidalMove(elevator,ElevatorConstants.maxSpeed,ElevatorConstants.maxAcceleration, ElevatorConstants.coralL4)); 

    SmartDashboard.putData("Reset Field Pose", new InstantCommand(() -> swerve.resetPose(new Pose2d())).ignoringDisable(true));
    // path_chooser = AutoBuilder.buildAutoChooserWithOptionsModifier("default", stream -> stream.filter(auto -> !auto.getName().startsWith(".")));
    // SmartDashboard.putData("auto", path_chooser);
    SmartDashboard.putData("auto", auto_chooser);
  }
  
  private void configureBindings() {
    operator.x().whileTrue(new AlgaeIntake(algaeI, -1)); // intake algae
    operator.a().whileTrue(new AlgaeIntake(algaeI, 1)); // shoot algae
    operator.y().whileTrue(new CoralShoot(coral, elevator,() -> 0.15));
    operator.b().toggleOnTrue(new AlgaeIntake(algaeI, -0.05));
    operator.axisGreaterThan(2, operatorRTDeadZone).whileTrue(new rawCoralSet(coral, -0.0, -0.20));
    operator.axisMagnitudeGreaterThan(5, operatorRSDeadZone).whileTrue(new ElevatorManual(elevator, () -> (-operator.getRightY() + 0.03)*0.2));
    operator.axisGreaterThan(3, operatorRTDeadZone).whileTrue(new CoralShoot(coral, elevator, () -> -0.20)); // outake
    /*
    operator.povDown().onTrue(new SetElevatorPID(elevator, ElevatorConstants.Min));
    operator.povLeft().onTrue(new SetElevatorPID(elevator, ElevatorConstants.triggerL2));
    operator.povRight().onTrue(new SetElevatorPID(elevator, ElevatorConstants.triggerL3));
    operator.povUp().onTrue(new SetElevatorPID(elevator, ElevatorConstants.triggerL4));
    */
    operator.povDown().onTrue(new ElevatorTrapezoidalMove(elevator, ElevatorConstants.maxSpeed, ElevatorConstants.maxAcceleration, ElevatorConstants.Min));
    operator.povLeft().onTrue(new ElevatorTrapezoidalMove(elevator, ElevatorConstants.maxSpeed, ElevatorConstants.maxAcceleration, ElevatorConstants.coralL2));
    operator.povRight().onTrue(new ElevatorTrapezoidalMove(elevator, ElevatorConstants.maxSpeed, ElevatorConstants.maxAcceleration, ElevatorConstants.coralL3));
    operator.povUp().onTrue(new ElevatorTrapezoidalMove(elevator, ElevatorConstants.maxSpeed, ElevatorConstants.maxAcceleration, ElevatorConstants.coralL4));
    operator.leftBumper().onTrue(new ElevatorTrapezoidalMove(elevator, ElevatorConstants.maxSpeed, ElevatorConstants.maxAcceleration, ElevatorConstants.algaeL2));
    operator.rightBumper().onTrue(new ElevatorTrapezoidalMove(elevator, ElevatorConstants.maxSpeed, ElevatorConstants.maxAcceleration, ElevatorConstants.algaeL3));
    /*
    operator.povUp().whileTrue(new PivotPid(algaeP, 180));
    operator.povDown().whileTrue(new PivotPid(algaeP, 0));
    */
    //driver.rightBumper().toggleOnTrue(new AllModulePID(swerve));
    driver.a().whileTrue(new MoveToReef(swerve, true));
    driver.b().onTrue(new InstantCommand(swerve::zeroHeading));
  }

  public Command getAutonomousCommand() {
    return auto_chooser.getSelected();
    // return path_chooser.getSelected();
  }
  public Command getTestCommand() {
    return null;
  }
}