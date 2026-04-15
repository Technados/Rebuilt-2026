// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Set;
import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ShooterConstants;
import frc.robot.shooting.ShotMap;

public class ShooterSubsystem extends SubsystemBase {

  private final SparkFlex leftShooterMotor;
  private final SparkFlex rightShooterMotor;
  private final SparkFlex centerShooterMotor;
  
  private final RelativeEncoder leftShooterEncoder;
  private final RelativeEncoder rightShooterEncoder;
  private final RelativeEncoder centerShooterEncoder;

  private SparkClosedLoopController shooterController;

  private double targetVelocity;
  private boolean shooterFeedLatched;

  public ShooterSubsystem() {
    // Initializes motors using constants and configs
    leftShooterMotor = new SparkFlex(ShooterConstants.kLeftShooterMotorCanId, MotorType.kBrushless);
    rightShooterMotor = new SparkFlex(ShooterConstants.kRightShooterMotorCanId, MotorType.kBrushless);
    centerShooterMotor = new SparkFlex(ShooterConstants.kCenterShooterMotorCanId, MotorType.kBrushless);

    leftShooterMotor.configure(
      Configs.ShooterSubsystem.leftShooterConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );
    rightShooterMotor.configure(
      Configs.ShooterSubsystem.rightShooterConfig,
      ResetMode.kResetSafeParameters,                                                   
      PersistMode.kPersistParameters
    );
    centerShooterMotor.configure(
      Configs.ShooterSubsystem.centerShooterConfig,
      ResetMode.kResetSafeParameters,                                                   
      PersistMode.kPersistParameters
    );
      
    // Creates encoders for both motors
    leftShooterEncoder = leftShooterMotor.getEncoder();
    rightShooterEncoder = rightShooterMotor.getEncoder();
    centerShooterEncoder = centerShooterMotor.getEncoder();
    
    // Creates a controller for right motor (left is a follower and will do what the right does)
    shooterController = rightShooterMotor.getClosedLoopController();
    
    targetVelocity = 0.0;
    shooterFeedLatched = false;
  }
    
  /*----------Getters----------*/
  
  public double getTargetVelocity() { // Returns the target velocity
    return targetVelocity;
  }
    
  public double getLeftVelocity() { // Returns the left motor velocity
    return leftShooterEncoder.getVelocity();
  }

  public double getRightVelocity() { // Returns the right motor velocity
    return rightShooterEncoder.getVelocity();
  }

  public double getCenterVelocity() { // Returns the center motor velocity
    return centerShooterEncoder.getVelocity();
  }
  
  /**
   * @return Returns true when all motors are within tolerance of target RPM
   */
  public boolean shooterAtVelocity() { 
    if (targetVelocity <= 1.0) return false;
    double tol = ShooterConstants.kShooterReadyToleranceRPM;
    return Math.abs(getLeftVelocity() - targetVelocity) <= tol
        && Math.abs(getRightVelocity() - targetVelocity) <= tol
        && Math.abs(getCenterVelocity() - targetVelocity) <= tol;  
  }

  public boolean shooterSafeToFeed() {
  if (targetVelocity <= 1.0) {
    shooterFeedLatched = false;
    return false;
  }

  double resumeThreshold = targetVelocity - ShooterConstants.kShooterReadyToleranceRPM;
  double dropThreshold = targetVelocity - ShooterConstants.kShooterFeedDropToleranceRPM;

  boolean allAtResume =
      getLeftVelocity() >= resumeThreshold
      && getRightVelocity() >= resumeThreshold
      && getCenterVelocity() >= resumeThreshold;

  boolean anyBelowDrop =
      getLeftVelocity() < dropThreshold
      || getRightVelocity() < dropThreshold
      || getCenterVelocity() < dropThreshold;

  if (!shooterFeedLatched) {
    // Do not begin feeding until the shooter is genuinely up to speed
    shooterFeedLatched = allAtResume;
  } else {
    // Once feeding has begun, allow some normal droop before cutting feed
    if (anyBelowDrop) {
      shooterFeedLatched = false;
    }
  }

  return shooterFeedLatched;
}

  /*----------Control Methods----------*/
  
  /**
   * Sets the velocity of the shooter motors. The preshooter is set to its rpm defined in the constants.
   * The left and right shooters are set the parameterized velocity.
   * @param velocity The velocity that the left and right shooter will run at.
   */
  public void setShooterVelocity(double velocity) { 
    // Clamp to your known safe range
    targetVelocity = MathUtil.clamp(velocity, 0.0, ShooterConstants.kShooterMaxRPM);
    
    // Only command the right shooter, left will follow (already set in configs)
    shooterController.setSetpoint(
      targetVelocity,
      ControlType.kVelocity,             
      ClosedLoopSlot.kSlot0
    );
  }
  
  /**
   * Sets the velocity of the shooters to their idle velocity.
   */
  public void idleShooter() { // temporarily disabled
    // Clamp to your known safe range
    double idleVelocity = MathUtil.clamp(ShooterConstants.kShooterIdleRPM, 0.0, ShooterConstants.kShooterMaxRPM);
    
    // Only command the right shooter, left will follow (already set in configs)
    shooterController.setSetpoint(
      idleVelocity,
      ControlType.kVelocity,             
      ClosedLoopSlot.kSlot0
    );
  }
  
  /**
   * Stops all of the motors.
   */
  public void stopShooter() {
  shooterFeedLatched = false;
  targetVelocity = 0.0;
  rightShooterMotor.set(0.0);
}
  
  /*----------Commands----------*/
  
  /**
   * @return Command to run the left and right motors idly
   */
  public Command idleShooterCommand() {
    return run(this::idleShooter);
  }

  /**
   * @param rpmSupplier Velocity that the left and right motors will run at
   * @return Command to set shooter velocity using {@link ShotMap}
   */
  public Command holdVelocityCommand(DoubleSupplier rpmSupplier) { 
    return runEnd(
      () -> setShooterVelocity(rpmSupplier.getAsDouble()),
      () -> stopShooter()
    );
  }

  /**
   * @param velocity Velocity that the left and right motors will run at
   * @return Command to set shooter velocity
   */
  public Command holdVelocityCommand(double velocity) { // Command to set shooter velocity
    return runEnd(
      () -> setShooterVelocity(velocity),
      () -> stopShooter()
    );
  }

  // grace period for shooter to keep running for 'time' if stopped unexpectedly (jam protect)
  public Command holdLastVelocityForCommand(double seconds) {
  return Commands.defer(
    () -> {
      double heldVelocity = targetVelocity;

      return this.runEnd(
        () -> {
          if (heldVelocity > 1.0) {
            shooterController.setSetpoint(
              heldVelocity,
              ControlType.kVelocity,
              ClosedLoopSlot.kSlot0
            );
          }
        },
        this::stopShooter
      ).withTimeout(seconds);
    },
    Set.of(this)
  );
}

  /*----------Periodic----------*/

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter/Target Velocity", getTargetVelocity());

    SmartDashboard.putNumber("Shooter/Left Velocity", getLeftVelocity());
    SmartDashboard.putNumber("Shooter/Right Velocity", getRightVelocity());
    SmartDashboard.putNumber("Shooter/Center Velocity", getCenterVelocity());
    
    SmartDashboard.putBoolean("Shooter/Shooter At Velocity", shooterAtVelocity());
    SmartDashboard.putBoolean("Shooter/Safe to Feed", shooterSafeToFeed());

    SmartDashboard.putNumber("Shooter/Right AppliedOutput", rightShooterMotor.getAppliedOutput());
    SmartDashboard.putNumber("Shooter/Right BusVoltage", rightShooterMotor.getBusVoltage());
    SmartDashboard.putNumber("Shooter/Right AppliedVolts", 
      rightShooterMotor.getAppliedOutput() * rightShooterMotor.getBusVoltage());
  }

}