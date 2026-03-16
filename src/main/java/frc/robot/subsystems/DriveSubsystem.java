// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.FieldConstants;
import frc.robot.Constants.AimConstants;


public class DriveSubsystem extends SubsystemBase {

  // Sensors and objects
  private final LEDSubsystem ledSubsystem;

  // The gyro sensor: NavX-2 Micro gyro from Kauai Labs
  // additional change: since using NavX-2 gyro, all getAngle calls in the drive sub system had to be changed to negative values
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

  private boolean slowMode = false; // Boolean flag to track slow mode

  private boolean traversalMode = false;
  private double traversalHeadingRad = Math.toRadians(45.0);


  private final Field2d field = new Field2d();
  private final SwerveDrivePoseEstimator m_poseEstimator;

  private final PIDController aimPid = 
    new PIDController(
      AimConstants.kAimP, 
      AimConstants.kAimI, 
      AimConstants.kAimD
    );

  // PathPlanner RobotConfig
  private RobotConfig config;

  private boolean hasFlashedEndgame = false;


  public DriveSubsystem(LEDSubsystem ledSubsystem) {

    SmartDashboard.putData("Field", field);

    // Load RobotConfig
    try {
        config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
        e.printStackTrace();
    }

    // Configure AutoBuilder at the end
    configureAutoBuilder();

    // Create robot poseEstimator that can take input from VisionSubsystem
    m_poseEstimator = // Currently has default StdDevs
      new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        getGyroRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()}, 
        new Pose2d(),
        VecBuilder.fill(0.05, 0.05, 0.01),
        VecBuilder.fill(0.8, 0.8, 0.5)
      );
    
    aimPid.enableContinuousInput(-Math.PI, Math.PI);

    this.ledSubsystem = ledSubsystem;

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

    // Add gyro info to dashboard
    SmartDashboard.putNumber("Gyro", getHeading()); // returns the heading of the robot and sends to dashboard
    SmartDashboard.putBoolean("Gyro Online", m_gyro.isConnected());

    var hubSupplier = (Supplier<Translation2d>) FieldConstants::getAllianceHub;

    SmartDashboard.putNumber("Distance from Hub",
      getPose().getTranslation().getDistance(hubSupplier.get()));

    // Flash LEDs red if gyro is offline
    if (!m_gyro.isConnected()) {
      ledSubsystem.flashOnceForGyroAlert(0.61, 2.0);
    }
    
    // Update pose estimator using wheel encoder and gyro information
    m_poseEstimator.update(
        getGyroRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        });

    // Set robot pose on field
    field.setRobotPose(getPose());

    // Flash LEDs blue at endgame(?)
    if (DriverStation.isTeleopEnabled() &&
      DriverStation.getMatchTime() <= 30.0 &&
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
    return m_poseEstimator.getEstimatedPosition();
  }

  public double getHeadingRadians() { // Gets the robot's heading
    return getPose().getRotation().getRadians();
  }

  public boolean isAimedAt(edu.wpi.first.math.geometry.Translation2d target) { // Returns true if robot is aimed at target
    var pose = getPose();
    double dx = target.getX() - pose.getX();
    double dy = target.getY() - pose.getY();
    double desired = Math.atan2(dy, dx);

    double errorRad = MathUtil.angleModulus(desired - pose.getRotation().getRadians());

    SmartDashboard.putBoolean("Aim/Is Aimed At", Math.abs(Math.toDegrees(errorRad)) <= AimConstants.kAimToleranceDeg);
    return Math.abs(Math.toDegrees(errorRad)) <= AimConstants.kAimToleranceDeg;
  }

  /**
  * Command the drivetrain to rotate toward a field point.
  * Translation is held at 0 here (stationary aiming).
  */
  public void aimAt(edu.wpi.first.math.geometry.Translation2d target) {
    var pose = getPose();
    double dx = target.getX() - pose.getX();
    double dy = target.getY() - pose.getY();
    double desired = Math.atan2(dy, dx);

    double omega = aimPid.calculate(pose.getRotation().getRadians(), desired);
    omega = omega * DriveConstants.kMaxSpeedMetersPerSecond;
    omega = MathUtil.clamp(omega, -AimConstants.kAimMaxOmegaRadPerSec, AimConstants.kAimMaxOmegaRadPerSec);

    // rotate in place, field-relative
    drive(0.0, 0.0, omega, true);

    if (isAimedAt(target)) {
      setX();
    }

  }
 
  // Adds measurement from the VisionSubsystem to the pose estimator
  public void addVisionMeasurement(Pose2d visonPose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
    m_poseEstimator.addVisionMeasurement(visonPose, timestampSeconds, stdDevs);
  }

  // Runs aimAt as a command
  public Command aimAtCommand(java.util.function.Supplier<edu.wpi.first.math.geometry.Translation2d> targetSupplier) {
    return run(() -> aimAt(targetSupplier.get()));
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(
        getGyroRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Enables/disables slow mode, turns bot to 45 degrees.
   * @param enable true if slow mode is enabled
   */
  public void setSlowMode(boolean enable) {
    slowMode = enable;
  }

  /**
   * Check if we need to enable slow mode based on conditions or driver input.
   * @param manualSlowMode true if driver is holding right bumper
   */
  public void updateDriveSlowMode(boolean manualSlowMode) {
    slowMode = manualSlowMode;
  }

  public void enableTraversalMode() {
    traversalMode = true;
    traversalHeadingRad = snapToNearestDiagonalRad(getPose().getRotation().getRadians());
  }

  public void disableTraversalMode() {
    traversalMode = false;
  }

  public boolean isTraversalMode() {
    return traversalMode;
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
                    xSpeedDelivered,
                    ySpeedDelivered,
                    rotDelivered,
                    getGyroRotation2d())
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

  public void driveRobotRelative(ChassisSpeeds speeds) { // Drives without using field relative
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public void driveTraversalAssist(double xSpeed, double ySpeed, boolean fieldRelative) {
    double rotCmdRadPerSec = aimPid.calculate(
      getPose().getRotation().getRadians(),
      traversalHeadingRad
    );

    rotCmdRadPerSec = MathUtil.clamp(
      rotCmdRadPerSec,
      -AimConstants.kAimMaxOmegaRadPerSec,
      AimConstants.kAimMaxOmegaRadPerSec
    );

    double rotInput = rotCmdRadPerSec * DriveConstants.kMaxAngularSpeed;
    rotInput = MathUtil.clamp(rotInput, -0.35, 0.35);

    double speedFactor = DriveConstants.kTraverseSpeedFactor;

    drive(
      xSpeed * speedFactor,
      ySpeed * speedFactor,
      rotInput,
      fieldRelative
    );

    SmartDashboard.putBoolean("Drive/Traversal Mode", traversalMode);
    SmartDashboard.putNumber("Drive/Traversal Heading Deg", Math.toDegrees(traversalHeadingRad));

  }


  public ChassisSpeeds getChassisSpeeds() { // Gets the speed of the chassis
    return DriveConstants.kDriveKinematics.toChassisSpeeds(new SwerveModuleState[] {
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_rearLeft.getState(),
      m_rearRight.getState()
    });
  }

  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /** Sets the wheels into an X formation to prevent movement. */
  public Command setXCommand() {
    return this.run(() -> setX());
  }

  public Command moveFixedDistanceCommand(double xMeters, double yMeters) { // Moves by a fixed x and y parameter
    return new InstantCommand(() -> {
        drive(xMeters, yMeters, 0, false); // Move in robot-relative space
    }, this).andThen(new InstantCommand(() -> drive(0, 0, 0, false), this)); // Stop after movement
  }



  public void stopMovement() { // Stop all movement
    drive(0, 0, 0, true); 
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
    return Math.IEEEremainder(getGyroRotation2d().getDegrees(), 360);
    //return Rotation2d.fromDegrees(m_gyro.getAngle()).getDegrees();
  }

  /**
  * Returns the gyro rotation in WPILib field convention:
  * - 0° = facing +X (toward red wall)
  * - CCW positive
  *
  * This should be the ONLY place we convert navX readings into a Rotation2d.
  * Everything else (pose estimator, driving, MT2 orientation) should use this.
  */
  private Rotation2d getGyroRotation2d() {
    // NavX angle commonly increases clockwise; WPILib expects CCW+.
    // Inverting is standard for navX in FRC.
    double angleDeg = -m_gyro.getAngle();

    // If you ever need to flip (wiring/mounting), do it ONE time here.
    if (DriveConstants.kGyroReversed) {
      angleDeg = -angleDeg;
    }

    return Rotation2d.fromDegrees(angleDeg);
  }


  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    // Keep turn rate consistent with getGyroRotation2d() (CCW+)
    double rate = -m_gyro.getRate();
    if (DriveConstants.kGyroReversed) rate = -rate;
      return rate;

  }

  private double snapToNearestDiagonalRad(double headingRad) {
    double[] diagonals = {
      Math.toRadians(45.0),
      Math.toRadians(135.0),
      Math.toRadians(225.0),
      Math.toRadians(315.0)
    };

    double best = diagonals[0];
    double bestErr = Math.abs(MathUtil.angleModulus(headingRad - best));

    for (double d : diagonals) {
      double err = Math.abs(MathUtil.angleModulus(headingRad - d));
      if (err < bestErr) {
        bestErr = err;
        best = d;
      }
    }

    return best;
  }

}
