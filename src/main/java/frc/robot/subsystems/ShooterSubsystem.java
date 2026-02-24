// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

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

    targetVelocity = -1;
  }

  public boolean shooterInVelocityRange(RelativeEncoder shooterEncoder) { // Returns true if motor velocity is in range, needs adjustment
    return 
      (targetVelocity - 100) < 
      shooterEncoder.getVelocity() 
      && 
      shooterEncoder.getVelocity() < 
      (targetVelocity + 100);
  }

  public boolean shooterAtVelocity() { // Returns true if the motors reach the target velocity
    return shooterInVelocityRange(leftShooterEncoder) && shooterInVelocityRange(rightShooterEncoder);
  }

  public void setShooterVelocity(double velocity) {
    // Clamp to your known safe range
    velocity = Math.max(0.0, Math.min(3500.0, velocity));

    // Returns if the velocity is already set to the target velocity
    if (targetVelocity == velocity) {
      return;
    }

    targetVelocity = velocity;

    leftShooterController.setReference(targetVelocity,
      ControlType.kMAXMotionVelocityControl);

    rightShooterController.setReference(targetVelocity,
      ControlType.kMAXMotionVelocityControl);
  }

  public Command runShooterCommand() { // Runs the main shooter
    return this.startEnd(
      () -> {
        setShooterVelocity(ShooterConstants.kShooterMotorVelocity);
      },
      () -> {
        setShooterVelocity(0.0);
      }
    );
  }

  public Command idleShooterCommand() { // Runs the main shooter at idle power
    return this.run(
      () -> {
        setShooterVelocity(ShooterConstants.kShooterMotorIdleVelocity);
      }
    );
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Velocity", leftShooterEncoder.getVelocity());
    SmartDashboard.putNumber("Right Velocity", rightShooterEncoder.getVelocity());
    SmartDashboard.putBoolean("shooterAtVelocity", shooterAtVelocity());
  }

}