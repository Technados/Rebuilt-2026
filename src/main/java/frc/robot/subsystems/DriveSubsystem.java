// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

//import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;


public class DriveSubsystem extends SubsystemBase {


// Sensors and objects
private final NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

private final LEDSubsystem ledSubsystem;

// The gyro sensor: NavX-2 Micro gyro from Kauai Labs
// additional change: since using NavX-2 gyro, all getAngle calls in the drive sub system had to be chnaged to negative values
// The NavX gyro is used to track the robot's orientation on the field.
private final AHRS m_gyro = new AHRS(SerialPort.Port.kUSB);


  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft =
      new MAXSwerveModule(
          DriveConstants.kFrontLeftDrivingCanId,
          DriveConstants.kFrontLeftTurningCanId,
          DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight =
      new MAXSwerveModule(
          DriveConstants.kFrontRightDrivingCanId,
          DriveConstants.kFrontRightTurningCanId,
          DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft =
      new MAXSwerveModule(
          DriveConstants.kRearLeftDrivingCanId,
          DriveConstants.kRearLeftTurningCanId,
          DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight =
      new MAXSwerveModule(
          DriveConstants.kRearRightDrivingCanId,
          DriveConstants.kRearRightTurningCanId,
          DriveConstants.kBackRightChassisAngularOffset);

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          DriveConstants.kDriveKinematics,
          Rotation2d.fromDegrees(-m_gyro.getAngle()),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
          });

      // PathPlanner RobotConfig
      private RobotConfig config;

      
      private boolean hasFlashedEndgame = false;


      public DriveSubsystem(LEDSubsystem ledSubsystem) {
  
        this.ledSubsystem = ledSubsystem;
   

          // Load RobotConfig
          try {
              config = RobotConfig.fromGUISettings();
          } catch (Exception e) {
              e.printStackTrace();
          }
  
          // Configure AutoBuilder at the end
          configureAutoBuilder();
      }
  
      private void configureAutoBuilder() {
          AutoBuilder.configure(
              this::getPose, // Robot pose supplier
              this::resetOdometry, // Method to reset odometry
              this::getChassisSpeeds, // Robot-relative ChassisSpeeds supplier
              (speeds, feedforwards) -> driveRobotRelative(speeds), // Drive robot
              new PPHolonomicDriveController( // Holonomic controller
                  new PIDConstants(8.5, 0.0, 0.11), // Translation PID
                  new PIDConstants(13.0, 0.0, 0.4)  // Rotation PID
              ),
              config, // RobotConfig loaded from PathPlanner GUI
              () -> {
                  // Flip paths for red alliance
                  var alliance = DriverStation.getAlliance();
                  return alliance.orElse(DriverStation.Alliance.Blue) != DriverStation.Alliance.Blue;
              },
              this // Subsystem requirements
          );
      }

      


  @Override
  public void periodic() {

    SmartDashboard.putNumber("Gyro", getHeading()); // returns the heading of the robot and sends to dashboard
    if (!m_gyro.isConnected()) {
      ledSubsystem.flashOnceForGyroAlert(0.61, 2.0); // 🔴 Red if gyro offline
  }
  


    
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(-m_gyro.getAngle()),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        });

        if (edu.wpi.first.wpilibj.DriverStation.isTeleopEnabled() &&
    edu.wpi.first.wpilibj.DriverStation.getMatchTime() <= 30.0 &&
    !hasFlashedEndgame) {

    ledSubsystem.flashPattern(0.87, 2.0); // 🔵 Blue flash for 2 seconds
    hasFlashedEndgame = true;
}

  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(-m_gyro.getAngle()),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        },
        pose);
  }

  private boolean slowMode = false; // Boolean flag to track slow mode

public void setSlowMode(boolean enable) {
    slowMode = enable;
}

/**
 * Check if we need to enable slow mode based on conditions or driver input.
 * @param manualSlowMode true if driver is holding right bumper
 */
public void updateDriveSlowMode(boolean manualSlowMode) {

    setSlowMode(manualSlowMode);
}

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

    SmartDashboard.putBoolean("Field Relative", fieldRelative);
    SmartDashboard.putBoolean("Slow Mode", slowMode);
    // Convert the commanded speeds into the correct units for the drivetrain
    // Apply speed reduction if slow mode is active
    // Apply speed reduction factor
    double speedFactor = slowMode ? DriveConstants.kSlowSpeedFactor : 1.0;

    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond * speedFactor;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond * speedFactor;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed * speedFactor; // Ensure rotation is also scaled

    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    -xSpeedDelivered,
                    -ySpeedDelivered,
                    rotDelivered,
                    Rotation2d.fromDegrees(-m_gyro.getAngle()))
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
    
    //Show Joystick input on DashBoard
    SmartDashboard.putNumber("xSpeed", xSpeed);
    SmartDashboard.putNumber("ySpeed", ySpeed);
    SmartDashboard.putNumber("rot", rot);
    
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
}

public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(new SwerveModuleState[] {
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState()
    });
}

  /** Sets the wheels into an X formation to prevent movement. */
  public Command setXCommand() {
    return this.run(
        () -> {
          m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
          m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
          m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
          m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        });
  }

  public Command moveFixedDistance(double xMeters, double yMeters) {
    return new InstantCommand(() -> {
        drive(xMeters, yMeters, 0, false); // Move in robot-relative space
    }, this).andThen(new InstantCommand(() -> drive(0, 0, 0, false), this)); // Stop after movement
}



  public void stopMovement() {
    drive(0, 0, 0, true); // Stop all movement

}


  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }
// The method below will set the gyro 180 deg adjusted angle to compensate for starting pose being bakward 
  public void resetGyroToFieldBackwards() {
    m_gyro.reset();
    m_gyro.setAngleAdjustment(180);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public Command zeroHeadingCommand() {
    return this.runOnce(() -> m_gyro.reset());
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(-m_gyro.getAngle()*(DriveConstants.kGyroReversed ? -1.0 : 1.0), 360);
    //return Rotation2d.fromDegrees(m_gyro.getAngle()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }



}
