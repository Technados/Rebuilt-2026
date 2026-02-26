// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  private final SparkFlex leftShooterMotor;
  private final RelativeEncoder leftShooterEncoder;
  private SparkClosedLoopController leftShooterController;
  
  private final SparkFlex rightShooterMotor;
  private final RelativeEncoder rightShooterEncoder;
  private SparkClosedLoopController rightShooterController;

  private double targetVelocity;

  public ShooterSubsystem() {
    // Initializes motors using constants and configs
    leftShooterMotor = new SparkFlex(ShooterConstants.kLeftShooterMotorCanId, MotorType.kBrushless);
    rightShooterMotor = new SparkFlex(ShooterConstants.kRightShooterMotorCanId, MotorType.kBrushless);

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

    leftShooterEncoder = leftShooterMotor.getEncoder();
    rightShooterEncoder = rightShooterMotor.getEncoder();

    leftShooterController = leftShooterMotor.getClosedLoopController();
    rightShooterController = rightShooterMotor.getClosedLoopController();

    targetVelocity = 0.0;
  }
  
  public void setShooterVelocity(double velocity) {
    // Clamp to your known safe range
    targetVelocity = MathUtil.clamp(velocity, 0.0, ShooterConstants.kShooterMaxRPM);
    
    // Optional feedforward (start at 0, tune later)
    double ffVolts = ShooterConstants.kShooterKSVolts + (ShooterConstants.kShooterKVVoltsPerRPM * targetVelocity);
    
    leftShooterController.setSetpoint(
      targetVelocity,
      ControlType.kVelocity,
      ClosedLoopSlot.kSlot0,
      ffVolts
      );
      
    rightShooterController.setSetpoint(
      targetVelocity,
      ControlType.kVelocity,
      ClosedLoopSlot.kSlot0,
      ffVolts
      );
  }
      
  public void idleShooter() { // Runs the main shooter at idle power - temporarily disabled
    //setShooterVelocity(ShooterConstants.kShooterIdleRPM);
    return;
  }
  
  public void stopShooter() { // Stops both motors
    leftShooterMotor.set(0.0);
    rightShooterMotor.set(0.0);
  }
  
  public double getTargetVelocity() { // Returns the target velocity
    return targetVelocity;
  }
  
  public double getLeftVelocity() { // Returns the velocity of the left motor
    return Math.abs(leftShooterEncoder.getVelocity());
  }
  
  public double getRightVelocity() { // Returns the velocity of the right motor
    return Math.abs(rightShooterEncoder.getVelocity());
  }
  
  /** True when both wheels are within tolerance of target RPM. */
  public boolean shooterAtVelocity() {
    if (targetVelocity <= 1.0) return false;
    double tol = ShooterConstants.kShooterReadyToleranceRPM;
    return Math.abs(getLeftVelocity() - targetVelocity) <= tol
        && Math.abs(getRightVelocity() - targetVelocity) <= tol;
  }
  
  // ---------------- Commands ----------------
  
  public Command idleShooterCommand() { // Command for the idle shooter
    return run(this::idleShooter);
  }

  public Command holdVelocityCommand(DoubleSupplier rpmSupplier) { // Command to set shooter velocity using ShotMap
    return run(
      () -> setShooterVelocity(rpmSupplier.getAsDouble())
    );
  }

  public Command holdVelocityCommand(double velocity) { // Command to set shooter velocity
    return runEnd(
      () -> setShooterVelocity(velocity),
      () -> stopShooter()
    );
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter/Target Velocity", getTargetVelocity());
    SmartDashboard.putNumber("Shooter/Left Velocity", getLeftVelocity());
    SmartDashboard.putNumber("Shooter/Right Velocity", getRightVelocity());
    SmartDashboard.putBoolean("Shooter/Shooter At Velocity", shooterAtVelocity());
  }

}