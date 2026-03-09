// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static final double kIntakeMotorPower = 0.65;

    public static final int kPivotMotorCanId = 11;
    public static final double kPivotIdleInPower = 0.03;
    public static final double kPivotIdleOutPower = -0.03;

    public static final double kPivotMechAdv = 52.89; // motor rev per arm rev

    public static final double kPivotEncoderTicksToDegrees = 360 / kPivotMechAdv;

    // Pivot closed-loop gains (start conservative)
    public static final double kPivotP = 0.0025;
    public static final double kPivotI = 0.0;
    public static final double kPivotD = 0.003;

    public static final double kPivotDegPerMotorRev = 360.0 / kPivotMechAdv; // ~6.8066

    public static final double kPivotDegPerSecPerMotorRPM = kPivotDegPerMotorRev / 60;

    public static final double kPivotPosToleranceDeg = 4;
    public static final double kPivotVelToleranceDegPerSec = 8.0;

    // MAXMotion limits (units follow velocityConversionFactor)
    public static final double kPivotCruiseArmDegPerSec = 500;
    public static final double kPivotAccelArmDegPerSec2 = 900;

    // Convert to what MaxMotion appears to want (MOTOR units)
    public static final double kPivotCruiseVelocity =
      kPivotCruiseArmDegPerSec / kPivotDegPerSecPerMotorRPM;  // ≈ 3085 motor RPM

    public static final double kPivotMaxAccel =
      kPivotAccelArmDegPerSec2 / kPivotDegPerSecPerMotorRPM;  // ≈ 7938 motor RPM per sec (approx)

  }

  public static final class FeederConstants {

    public static final int kFeederMotorCanId = 12;
    public static final double kFeederMotorPower = .90;

    public static final int kPreShooterMotorCanId = 13;
    public static final double kPreShooterMotorPower = .90;
    
  }
  
  public static final class ShooterConstants {
    
    public static final int kLeftShooterMotorCanId = 14;
    public static final int kRightShooterMotorCanId = 15;

    public static final double kShooterMaxRPM = 5000.0; 
    public static final double kShooterIdleRPM = 1500.0; 
    
    // Must be adjusted before testing!
    public static final double kShooterP = 0.0003;
    public static final double kShooterI = 0.0;
    public static final double kShooterD = 0.0;
    
    // “At speed” tolerance used to decide when we are allowed to feed.
    public static final double kShooterReadyToleranceRPM = 250.0;

    // Optional feedforward: volts = kS + kV * RPM
    // Start at 0 to prove closed-loop works, then add feedforward later for better recovery.
    public static final double kShooterKSVolts = 0.10; //Calculated from direct shooter data by nitzky
    public static final double kShooterKVVoltsPerRPM = 1.0 / 6784; // ~0.00177 (recip. of motors Kv)

  }

  public static final class AimConstants {
    public static final double kAimP = 0.4; // radians -> rad/s-ish output scaling (tune)
    public static final double kAimMaxOmegaRadPerSec = 3.0;
    public static final double kAimToleranceDeg = 2.0;
  }


  public static final class HoodConstants {

    public static final int kLeftHoodServoPwm = 0;
    public static final int kRightHoodServoPwm = 1;

    // Clamp to protect mechanics (tune after first test)
    public static final double kMinPos = 0.2;
    public static final double kMaxPos = 0.68;

    public static final double kPosTolerance = 0.01;

    // Modeled servo speed (pos units per second). This is not physics-accurate;
    // it's a practical way to have a stable "hood is ready" signal.
    public static final double kMaxPosUnitsPerSec = 0.80;

  }

  public static final class ClimberConstants { // All constants must be changed before use

    public static final int kClimberMotorCanId = -1;

    public static final double kClimberMechAdv = 0;

    // Must be adjusted before testing!
    public static final double kClimberP = 0.0;
    public static final double kClimberI = 0.0;
    public static final double kClimberD = 0.0;

    public static final double kClimberPosToleranceDeg = 0.0;
    public static final double kClimberVelToleranceDegPerSec = 0.0;

    public static final double kClimberDegPerMotorRev = 360.0 / kClimberMechAdv;
    public static final double kClimberDegPerSecPerMotorRPM = kClimberDegPerMotorRev / 60;

  }

  public static final class LEDConstants {

    public static final int kLEDPwmPort = 2;

  }

  public static final class LimelightPID {
    // 🎯 PID Gains for LL Targeting

    public static final double kP_turn = 0.2;   // Previously 0.1
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
    public static final String kFrontLimelightName = "limelight-front";
    public static final String kBackLimelightName = "limelight-back";

    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0.8, 0.8, Math.toRadians(10));
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.3, 0.3, Math.toRadians(5));

    public static final double kMaxAcceptedPoseJumpMeters = 2.0;

  }

  public static class TestPoses {
    public static final Pose2d kTestStartPose = new Pose2d(12.928, 4, Rotation2d.fromDegrees(180));
    
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 3;
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
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = VortexConstants.kFreeSpeedRpm / 60;
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

  public static final class VortexConstants {
    public static final double kFreeSpeedRpm = 6784;
  }


}
