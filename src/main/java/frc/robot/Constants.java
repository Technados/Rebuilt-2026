// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

/*
 * Port Numbers:
 * 
 * PDH: CAN ID 9
 * 
 * Drive Motors (4)
 *    Left Front: CAN ID 1, PDP 10
 *    Right Front: CAN ID 2, PDP 9
 *    Left Rear: CAN ID 3, PDP 12
 *    Right Rear: CAN ID 4, PDP 6
 * 
 * Steering Motors (4)
 *    Left Front: CAN ID 5, PDP 11
 *    Right Front: CAN ID 6, PDP 8
 *    Left Rear: CAND ID 7, PDP 13
 *    Right Rear: CAN ID 8, PDP 5
 * 
 * Intake Motor: CAN ID 10, PDP [PORT]
 * Pivot Intake: CAND ID 11, PDP 7
 * 
 * Feeder Motor: CAN ID 12, PDP 14
 * 
 * Pre Shooter Motor: CAN ID 13, PDP 15
 * Left Shooter Motor: CAN ID 14, PDP 16
 * Right Shooter Motor: CAN ID 15, PDP 17
 */

public final class Constants {

  public static final class IntakeConstants {
    public static final int kIntakeMotorCanId = 10;
    public static final double kIntakeMotorPower = -0.75;

    public static final int kPivotMotorCanId = 11;
    public static final double kPivotMotorPower = 0.1;
    // public static final double kPivotMotorExtended = //Degrees;

  }

  public static final class FeederConstants {
    public static final int kFeederMotorCanId = 12;
    public static final double kFeederMotorPower = 0.2;
  }

  public static final class ShooterConstants {
    public static final int kPreShooterMotorCanId = 13;
    public static final double kPreShooterMotorPower = -0.2;

    public static final int kLeftShooterMotorCanId = 14;
    public static final double kLeftShooterMotorPower = -0.2;

    public static final int kRightShooterMotorCanId = 15;
    public static final double kRightShooterMotorPower = 0.2;
  }

  public static final class LimelightPID {
    // 🎯 PID Gains for LL Targeting

    public static final double kP_turn = 0.15;   // Previously 0.1
    public static final double kI_turn = 0.000;
    public static final double kD_turn = 0.0;   // New (small D gain)

    public static final double kP_distance = 0.2; // Previously 0.15
    public static final double kI_distance = 0.0;
    public static final double kD_distance = 0.0; // New (small D gain)

    public static final double kP_strafe = 0.15;  // Previously 0.15
    public static final double kI_strafe = 0.0;
    public static final double kD_strafe = 0.0005; // New (small D gain)

  }

  public static final class VisionConstants {
    public static final String kFrontLimelightName = "front-limelight";
    public static final String kBackLimelightName = "back-limelight";

    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0.8, 0.8, Math.toRadians(10));
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.3, 0.3, Math.toRadians(5));

    public static final double kMaxAcceptedPoseJumpMewters = 2.0;

  }

  public static class TestPoses {
    public static final Pose2d kTestStartPose = new Pose2d(14, 4, Rotation2d.fromDegrees(180));
    
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 7;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second
    public static final double kSlowSpeedFactor = 0.20; // Slow mode speed factor (40% of normal speed)

    // Chassis configuration
    public static final double kTrackWidth 
    = Units.inchesToMeters(21.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(21.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kRearLeftDrivingCanId = 3;
    public static final int kFrontRightDrivingCanId = 2;
    public static final int kRearRightDrivingCanId = 4;

    public static final int kFrontLeftTurningCanId = 5;
    public static final int kRearLeftTurningCanId = 7;
    public static final int kFrontRightTurningCanId = 6;
    public static final int kRearRightTurningCanId = 8;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 12;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction =
        (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps =
        (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.1;
    public static final double kTriggerButtonThreshold = 0.2;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }


}
