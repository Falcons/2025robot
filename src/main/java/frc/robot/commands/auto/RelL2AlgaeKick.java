// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.algae.PivotPid;
import frc.robot.commands.coral.RawShootForTime;
import frc.robot.commands.moveToTarget.UnfilterdFollowTagG;
import frc.robot.subsystems.algae.Intake;
import frc.robot.subsystems.algae.Pivot;
import frc.robot.subsystems.driveTrain.SwerveSubsystem;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.shooter.Coral;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RelL2AlgaeKick extends SequentialCommandGroup {
    /** Creates a new UnfilteredRelLimeL1.*/
  public RelL2AlgaeKick(SwerveSubsystem swerve, Elevator elevator, Pivot pivot,Intake intake, Coral coral, double lOrR) {
    // Add your commands in the addCommands() call, e.g.
    addCommands( 
      new Taxi(swerve, 0.5),
      new UnfilterdFollowTagG(swerve),
      new relAutoDrive(swerve, new ChassisSpeeds(0.5, 0, 0), 0.5),
      new PivotPid(pivot, AlgaeConstants.pivotKick),
      new ElevatorAndAlgae(elevator, intake, ElevatorConstants.coralL2).asProxy(),
      new relAutoDrive(swerve, new ChassisSpeeds(-0.25, 0, 0), 0.15),
      new relAutoDrive(swerve, new ChassisSpeeds(0, lOrR, 0), 0.31),
      new relAutoDrive(swerve, new ChassisSpeeds(0.5, 0, 0), 0.5),
      new RawShootForTime(coral, -0.30, -0.30, 2).asProxy()
     );
  }
}
