// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Limelights;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightTag extends SubsystemBase {
  /** Creates a new LimelightTag. */
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry targetpose_robotspace = table.getEntry("targetpose_robotspace");
  public LimelightTag() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //read values periodically
    double v = tv.getDouble(0.0);
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    double[] targetPose = targetpose_robotspace.getDoubleArray(new double[6]);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("see target", v);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumberArray("position of apriltag", targetPose);
  }
  /** Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees / LL2: -29.8 to 29.8 degrees) */
  public double getX(){
    return tx.getDouble(0.0);
  }
  /** Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees / LL2: -24.85 to 24.85 degrees) */
  public double getY(){
    return ty.getDouble(0.0);
  }
  /** Target Area (0% of image to 100% of image) */
  public double Area(){
    return ta.getDouble(0.0);
  }
  public double getTargetPoseX(){
    double[] targetPoseX = targetpose_robotspace.getDoubleArray(new double[6]);
    return targetPoseX[0];
  }
  public double getTargetPoseY(){
    double[] targetPoseY = targetpose_robotspace.getDoubleArray(new double[6]);
    return targetPoseY[2];
  }
  public boolean hasTarget(){
    return tv.getDouble(0.0) == 1;
  }
}
