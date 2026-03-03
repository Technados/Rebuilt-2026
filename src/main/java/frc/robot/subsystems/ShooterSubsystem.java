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
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  private final SparkFlex leftShooterMotor;
  private final SparkFlex rightShooterMotor;
  private final SparkMax preShooterMotor;
  
  private final RelativeEncoder leftShooterEncoder;
  private final RelativeEncoder rightShooterEncoder;
  private final RelativeEncoder preShooterEncoder;

  private SparkClosedLoopController rightShooterController;
  private SparkClosedLoopController preShooterController;

  private double targetVelocity;

  public ShooterSubsystem() {
    // Initializes motors using constants and configs
    leftShooterMotor = new SparkFlex(ShooterConstants.kLeftShooterMotorCanId, MotorType.kBrushless);
    rightShooterMotor = new SparkFlex(ShooterConstants.kRightShooterMotorCanId, MotorType.kBrushless);
    preShooterMotor = new SparkMax(ShooterConstants.kPreShooterMotorCanId, MotorType.kBrushless);

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
    preShooterMotor.configure(
      Configs.ShooterSubsystem.preShooterConfig,
      ResetMode.kResetSafeParameters,                                                   
      PersistMode.kPersistParameters
      );
      
      // Creates encoders for both motors
      leftShooterEncoder = leftShooterMotor.getEncoder();
      rightShooterEncoder = rightShooterMotor.getEncoder();
      preShooterEncoder = preShooterMotor.getEncoder();
      
      // Creates a controller for right motor (left is a follower and will do what the right does)
      rightShooterController = rightShooterMotor.getClosedLoopController();
      preShooterController = preShooterMotor.getClosedLoopController();
      
      targetVelocity = 0.0;
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

  public double getPreVelocity() { // Returns the preshooter motor velocity
    return preShooterEncoder.getVelocity();
  }
  
  public boolean shooterAtVelocity() {  // Returns true when both wheels are within tolerance of target RPM
    if (targetVelocity <= 1.0) return false;
    double tol = ShooterConstants.kShooterReadyToleranceRPM;
    return (getLeftVelocity() - targetVelocity) <= tol
        && (getRightVelocity() - targetVelocity) <= tol
        && (getPreVelocity() - targetVelocity) <= tol;
  }

  /*----------Control Methods----------*/
  
  public void setShooterVelocity(double velocity) { // Sets the velocity of both shooters
    // Clamp to your known safe range
    targetVelocity = MathUtil.clamp(velocity, 0.0, ShooterConstants.kShooterMaxRPM);
    
    // Only command the right shooter, left will follow (already set in configs)
    rightShooterController.setSetpoint(
      targetVelocity,
      ControlType.kVelocity,             
      ClosedLoopSlot.kSlot0
    );

    // Sets the preshooter to its rpm defined in the constants
    preShooterController.setSetpoint(
      ShooterConstants.kPreShooterMotorRPM, 
      ControlType.kVelocity
    );
  }
  
  public void idleShooter() { // Sets the velocity of both shooters to their idle velocity - temporarily disabled
    //setShooterVelocity(ShooterConstants.kShooterIdleRPM);
  }
  
  public void stopShooter() { // Stops both motors - Still needs testing!
    rightShooterMotor.set(0.0);
  }
  
  /*----------Commands----------*/
  
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

  /*----------Periodic----------*/

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter/Target Velocity", getTargetVelocity());
    SmartDashboard.putNumber("Shooter/Left Velocity", getLeftVelocity());
    SmartDashboard.putNumber("Shooter/Right Velocity", getRightVelocity());
    SmartDashboard.putNumber("Shooter/Pre Velocity", getPreVelocity());
    SmartDashboard.putBoolean("Shooter/Shooter At Velocity", shooterAtVelocity());

    SmartDashboard.putNumber("Shooter/Right AppliedOutput", rightShooterMotor.getAppliedOutput());
    SmartDashboard.putNumber("Shooter/Right BusVoltage", rightShooterMotor.getBusVoltage());
    SmartDashboard.putNumber("Shooter/Right AppliedVolts", 
      rightShooterMotor.getAppliedOutput() * rightShooterMotor.getBusVoltage());
  }

}