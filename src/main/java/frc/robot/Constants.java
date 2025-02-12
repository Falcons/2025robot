// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public final class Constants {

    public static final class ModuleConstants {
        public static final double neoFreeSpeedRPM = 5820;
        public static final int sparkMaxDataPort = -1;
        
        //Drive Motor
        public static final double driveMotorRotToMetre = 14.0/50.0 * 27.0/17.0 * 15.0/45.0 * 4 * Math.PI / 39.3;
        public static final double driveMotorRPMToMetresPerSecond = driveMotorRotToMetre / 60.0;

        //4.60 m/s
        public static final double driveMaxSpeedMPS = neoFreeSpeedRPM * driveMotorRPMToMetresPerSecond;
        public static final double slowModeSpeed = driveMaxSpeedMPS * 0.2;

        //Turning Motor

        public static final double turningRotToWheelDegree = 1.0 / (150.0 / 7.0) * 360;
        public static final double turningRPMToDegreePerSecond = turningRotToWheelDegree / 60.0;

        public static final double turningMotorRotToRadian = 1.0 / (150.0 / 7.0) * 2 * Math.PI;
        public static final double turningMotorRPMToRadianPerSecond = turningMotorRotToRadian / 60.0;

        //Absolute Encoder
        public static final double voltToDegree = 360.0 / 3.3;
        public static final double voltToRad = 2 * Math.PI / 3.3;
        
        //Front Left
        public static final int frontLeftDriveCANID = 4;
        public static final int frontLeftTurningCANID = 3;
        public static final boolean frontLeftReversed = true;
        public static final double frontLeftAbsoluteOffset = 221.86;
        public static final double frontLeftTurningkP = 0.0035;
        public static final double frontLeftTurningkI = 0.05;

        //Front Right
        public static final int frontRightDriveCANID = 7;
        public static final int frontRightTurningCANID = 8;
        public static final boolean frontRightReversed = true;
        public static final double frontRightAbsoluteOffset = 152.70;
        public static final double frontRightTurningkP = 0.004;
        public static final double frontRightTurningkI = 0.05;

        //Back Left
        public static final int backLeftDriveCANID = 2;
        public static final int backLeftTurningCANID = 1;
        public static final boolean backLeftReversed = true;
        public static final double backLeftAbsoluteOffset = 72.73;
        public static final double backLeftTurningkP = 0.004;
        public static final double backLeftTurningkI = 0.05;

        //Back Right
        public static final int backRightDriveCANID = 5;
        public static final int backRightTurningCANID = 6;
        public static final boolean backRightReversed = true;
        public static final double backRightAbsoluteOffset = Units.radiansToDegrees(1.639);//95.95;
        public static final double backRightTurningkP = 0.004;
        public static final double backRightTurningkI = 0.05;
    }

    public static final class DriveConstants {
        public static final int pigeonCANID = 12;
        public static final double kDeadband = 0.05;

        public static final double kTrackwidth = Units.inchesToMeters(23.75);
            //Math to get Max Angular Speed
            private static final double rotDiameter = Math.sqrt(2 * kTrackwidth * kTrackwidth);
            private static final double rotCircumference = rotDiameter * Math.PI;
            private static final double secondsForOneRot = rotCircumference / ModuleConstants.driveMaxSpeedMPS;
            private static final double maxAngularSpeedRotPerSecond = 1.0 / secondsForOneRot;
        public static final double maxAngularSpeedRadiansPerSecond = maxAngularSpeedRotPerSecond * 2 * Math.PI;
        //10.78 rad/s
        //617.56 deg/s

        public static final double driveBaseRadius = rotDiameter / 2.0;

        //FL, FR, BL, BR
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kTrackwidth / 2.0, kTrackwidth / 2.0),
            new Translation2d(kTrackwidth / 2.0, -kTrackwidth / 2.0),
            new Translation2d(-kTrackwidth / 2.0, kTrackwidth / 2.0),
            new Translation2d(-kTrackwidth / 2.0, -kTrackwidth / 2.0));

        public static final double translationKP = 2.3;
        public static final double translationKI = 0;
        public static final double translationKD = 0;

        public static final double rotationKP = 2.5;
        public static final double rotationKI = 1.2;
        public static final double rotationKD = 0;

    }
    //TODO: make not placeholder
    public static final class ShooterConstants {
        public static final int leftMoterCANID = 1111;
        public static final int rightMoterCANID = 1112;
    }
    public static final class AlgaeConstants {
        public static final int pivotMoterCANID = 1113;
        public static final int intakeMoterCANID = 1114;
        public static final double motarRatio = 85.5; //85.5:1 wtf kind of ratio is that
        public static final double slowModeSpeed = 0.2;
        public static final double pivotMotorRotToDegree = motarRatio * 360;
        public static final double algeaPivotSetpointL1 = 0; 
        public static final double algeaPivotSetpointL2 = 0; 
        public static final double algeaPivotSetpointL3 = 0; 
        public static final double algeaPivotSetpointL4 = 0; 
    }
    public static final class ElevatorConstants {
        public static final int motarRatio = 12; //12:1 once i find the proper variable type
        public static final int liftMoter1CANID = 1115;
        public static final int liftMoter2CANID = 1116;
        public static final int TOFTopCANID = 1117;
        public static final double motorRotToMM = motarRatio * (2 * Math.PI) / 1234; //ratio * 2pi / gear radius in millimeters
        //note: elevator moves 2 inchs for every inch the chain moves(this ^ gives chain atm) -madness
        public static final double TOFMin = 0; //Alexa play despacito - ai chan //wtf is this autofill/bot comment -madness
        public static final double TOFSlowModetrigger = 305;
        public static final double TOFMax = 0; 
        public static final double TOFEdgeBuffer = 100;
        /**array [min, max]*/public static final double[] TOFTriggerL1 = {0,0}; 
        /**array [min, max]*/public static final double[] TOFTriggerL2 = {0,0};
        /**array [min, max]*/public static final double[] TOFTriggerL3 = {0,0};
        /**array [min, max]*/public static final double[] TOFTriggerL4 = {0,0};
        public static final double slowModeSpeed = 0.2; 
    }
    public static final class airlockConstants {
        public static final int backTOFCANID = 1118;
        public static final int frontTOFCANID = 1119;
        /**array [min, max]*/public static final double[] frontTOFTrigger = {0,0};
        /**array [min, max]*/public static final double[] backTOFTrigger = {0,0};
    }
}