// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import au.grapplerobotics.CanBridge;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.algae.AlgaePivot;
import frc.robot.commands.algae.ElevatorAndPivotOut;
import frc.robot.commands.algae.ElevatorLowAndPivotOut;
import frc.robot.commands.algae.IntakeForTime;
import frc.robot.commands.auto.DRL1;
import frc.robot.commands.auto.PIDtest;
import frc.robot.commands.auto.Taxi;
import frc.robot.commands.auto.UnfilterdRelLimeL1;
import frc.robot.commands.auto.UnfilterdRelLimeL2;
import frc.robot.commands.algae.pivotPidToggle;
import frc.robot.commands.algae.AlgaeIntake;
import frc.robot.commands.coral.CoralShoot;
import frc.robot.commands.coral.CoralStep;
import frc.robot.commands.coral.rawCoralSet;
import frc.robot.commands.driveTrain.AllModulePID;
import frc.robot.commands.driveTrain.SetHeading;
import frc.robot.commands.driveTrain.SwerveJoystick;
import frc.robot.commands.driveTrain.SwerveSlowModeHold;
import frc.robot.commands.driveTrain.invertdrive;
import frc.robot.commands.elevator.ElevatorHoldVoltage;
import frc.robot.commands.elevator.ElevatorManual;
import frc.robot.commands.elevator.ElevatorTrapezoidalMove;
import frc.robot.commands.elevator.PivotAndElevatorHome;
import frc.robot.commands.moveToTarget.PIDFollowTag;
import frc.robot.subsystems.Airlock;
import frc.robot.subsystems.FalconFlare;
import frc.robot.subsystems.algae.Pivot;
import frc.robot.subsystems.shooter.Coral;
import frc.robot.subsystems.driveTrain.SwerveSubsystem;
import frc.robot.subsystems.elevator.Elevator;
public class RobotContainer {

  private final Airlock airlock = new Airlock();
  private final FalconFlare flare = new FalconFlare();
  private final Elevator elevator = new Elevator(airlock, flare);
  private final Coral coral = new Coral();
  private final Pivot algaeP = new Pivot(elevator);
  private final frc.robot.subsystems.algae.Intake algaeI = new frc.robot.subsystems.algae.Intake();
  private final SwerveSubsystem swerve = new SwerveSubsystem();
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);


  private final double globalSpeedMod = 1;
  private final double operatorRSDeadZone = 0.2;
  private final double operatorLSDeadZone = 0.1;
  private final double operatorRTDeadZone = 0.01;
  private final double operatorLTDeadZone = 0.01;
  // private final Boolean layout = false;
  // SendableChooser<Command> path_chooser = new SendableChooser<Command>();
  SendableChooser<Command> auto_chooser = new SendableChooser<Command>();
  public RobotContainer() { 
    System.out.println("robot start");
    swerve.zeroHeading();
    CanBridge.runTCP();
    swerve.setDefaultCommand(new SwerveJoystick(
      swerve, 
      () -> -driver.getLeftY(), 
      () -> -driver.getLeftX(), 
      () -> -driver.getRightX(), 
      () -> !driver.getHID().getLeftBumper()));
      coral.setDefaultCommand(new CoralStep(coral, airlock, () -> -0.1));
      // algaeP.setDefaultCommand(new pivotDefault(algaeP, elevator));
      // algaeP.setDefaultCommand(new AlgaePivot(algaeP, () -> operator.getLeftY()*0.2)); // pivot
      // algaeI.setDefaultCommand(new intakeVoltage(algaeI, () -> 5.0));
    elevator.setDefaultCommand(new ElevatorHoldVoltage(elevator));

    configureBindings();

    NamedCommands.registerCommand("intake algae", new AlgaeIntake(algaeI, 1*globalSpeedMod));
    NamedCommands.registerCommand("outTake algae", new IntakeForTime(algaeI, -1*globalSpeedMod, 0.5));
    NamedCommands.registerCommand("outTake coral", new CoralShoot(coral, elevator,() -> 1.0));
    NamedCommands.registerCommand("set elevator bottom", new ElevatorTrapezoidalMove(elevator, ElevatorConstants.maxSpeed, ElevatorConstants.maxAcceleration, ElevatorConstants.Min));
    NamedCommands.registerCommand("set elevator L1", new ElevatorTrapezoidalMove(elevator,ElevatorConstants.maxSpeed,ElevatorConstants.maxAcceleration, ElevatorConstants.coralL1));
    NamedCommands.registerCommand("set elevator L2", new ElevatorTrapezoidalMove(elevator,ElevatorConstants.maxSpeed,ElevatorConstants.maxAcceleration, ElevatorConstants.coralL2));
    NamedCommands.registerCommand("set elevator L3", new ElevatorTrapezoidalMove(elevator,ElevatorConstants.maxSpeed,ElevatorConstants.maxAcceleration, ElevatorConstants.coralL3));
    NamedCommands.registerCommand("set elevator L4", new ElevatorTrapezoidalMove(elevator,ElevatorConstants.maxSpeed,ElevatorConstants.maxAcceleration, ElevatorConstants.coralL4M)); 

    SmartDashboard.putData("Reset Field Pose", new InstantCommand(() -> swerve.resetPose(new Pose2d())).ignoringDisable(true));
    // path_chooser = AutoBuilder.buildAutoChooserWithOptionsModifier("default", stream -> stream.filter(auto -> !auto.getName().startsWith(".")));
    // SmartDashboard.putData("auto", path_chooser);
    auto_chooser.setDefaultOption("taxi", new Taxi(swerve, 2.0));
    auto_chooser.addOption("dead L1", new DRL1(swerve, elevator, coral, algaeP));
    auto_chooser.addOption("L1", new UnfilterdRelLimeL1(swerve, elevator, algaeP, coral));
    // auto_chooser.addOption("L2 left", new UnfilterdRelLimeL2(swerve, elevator, algaeP, coral, 1));
    auto_chooser.addOption("L2 right", new UnfilterdRelLimeL2(swerve, elevator, algaeP, coral, -1));
    /*
    auto_chooser.addOption("Red right L1", new relLimeL1(swerve, elevator, coral, algaeP, 9));
    auto_chooser.addOption("Red back L1", new relLimeL1(swerve, elevator, coral, algaeP, 10));
    auto_chooser.addOption("Red left L1", new relLimeL1(swerve, elevator, coral, algaeP, 11));
    auto_chooser.addOption("blue right L1", new relLimeL1(swerve, elevator, coral, algaeP, 22));
    auto_chooser.addOption("blue back L1", new relLimeL1(swerve, elevator, coral, algaeP, 21));
    auto_chooser.addOption("blue left L1", new relLimeL1(swerve, elevator, coral, algaeP, 20));
    */
    SmartDashboard.putData("auto", auto_chooser);
  }
  
  private void configureBindings() {
    operator.x().whileTrue(new AlgaeIntake(algaeI, -1)); // intake algae
    operator.a().whileTrue(new AlgaeIntake(algaeI, 1)); // shoot algae
    // operator.y().whileTrue(new CoralShoot(coral, elevator,() -> 0.15));
    operator.y().onTrue(new PivotAndElevatorHome(algaeP, elevator));
    operator.b().toggleOnTrue(new AlgaeIntake(algaeI, -0.08));
    operator.axisGreaterThan(2, operatorLTDeadZone).whileTrue(new rawCoralSet(coral, -0.05, -0.20));
    operator.axisMagnitudeGreaterThan(5, operatorRSDeadZone).whileTrue(new ElevatorManual(elevator, swerve, () -> (-operator.getRightY() + 0.03)*0.2));
    operator.axisMagnitudeGreaterThan(1, operatorLSDeadZone).whileTrue(new AlgaePivot(algaeP, () -> (-operator.getLeftY())*0.30));
    operator.axisGreaterThan(3, operatorRTDeadZone).whileTrue(new CoralShoot(coral, elevator, () -> -0.30)); // outake
    
    
    operator.povDown().onTrue(new ElevatorTrapezoidalMove(elevator, ElevatorConstants.maxSpeed, ElevatorConstants.maxAcceleration, ElevatorConstants.coralL1));
    operator.povLeft().onTrue(new ElevatorTrapezoidalMove(elevator, ElevatorConstants.maxSpeed, ElevatorConstants.maxAcceleration, ElevatorConstants.coralL2));
    operator.povRight().onTrue(new ElevatorTrapezoidalMove(elevator, ElevatorConstants.maxSpeed, ElevatorConstants.maxAcceleration, ElevatorConstants.coralL3));
    operator.povUp().onTrue(new ElevatorLowAndPivotOut(elevator, algaeP));
    operator.leftBumper().onTrue(new ElevatorAndPivotOut(algaeP, elevator, ElevatorConstants.algaeL2));
    operator.rightBumper().onTrue(new ElevatorAndPivotOut(algaeP, elevator, ElevatorConstants.algaeL3));
    
    /* 
    operator.start().onTrue(new PivotPid(algaeP, AlgaeConstants.pivotMax));
    operator.back().onTrue(new PivotPid(algaeP, AlgaeConstants.pivotOut));
    */
    operator.back().onTrue(new ElevatorAndPivotOut(algaeP, elevator, ElevatorConstants.coralL1));
    operator.start().onTrue(new pivotPidToggle(algaeP));
    
    //driver.rightBumper().toggleOnTrue(new AllModulePID(swerve));
    // driver.a().onTrue(new SwerveToggleSlowMode(swerve));
    // driver.x().whileTrue(new pathToTag(swerve, 6));
    // driver.x().whileTrue(new L1ToSoruce(swerve, elevator, coral, 11));
    // driver.start().whileTrue(new FollowTagG(swerve, 11));
    // driver.back().whileTrue(new relLimeL1(swerve, elevator, coral, 11));
    driver.rightBumper().whileTrue(new SwerveSlowModeHold(swerve, elevator));
    driver.y().onTrue(new invertdrive(swerve));
    driver.b().onTrue(new InstantCommand(swerve::zeroHeading));
    // driver.x().whileTrue(new AllModulePID(swerve));
    // driver.x().whileTrue(new PIDtest(swerve, 1));
    driver.x().onTrue(new PIDFollowTag(swerve, 6));
    /*
    driver.povUp().onTrue(new SetElevatorSmallPIDMan(elevator, +0.5));
    driver.povDown().onTrue(new SetElevatorSmallPIDMan(elevator, -0.5));
    */
    driver.povUp().onTrue(new ElevatorTrapezoidalMove(elevator, ElevatorConstants.maxSpeed, ElevatorConstants.maxAcceleration, ElevatorConstants.coralL4H));
    driver.povRight().onTrue(new ElevatorTrapezoidalMove(elevator, ElevatorConstants.maxSpeed, ElevatorConstants.maxAcceleration, ElevatorConstants.coralL4M));
    driver.povDown().onTrue(new ElevatorTrapezoidalMove(elevator, ElevatorConstants.maxSpeed, ElevatorConstants.maxAcceleration, ElevatorConstants.coralL4L));
  }

  public Command getAutonomousCommand() {
    try {
      System.out.println("starting " + auto_chooser.getSelected().getName());
      return auto_chooser.getSelected();
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return new Taxi(swerve, 2.0);
    }
  }
  public Command getTestCommand() {
    return null;
  }
}