// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  private final SparkMax preShooterMotor;

  private final SparkFlex leftShooterMotor;
  private final RelativeEncoder leftShooterEncoder;

  private final SparkFlex rightShooterMotor;
  private final RelativeEncoder rightShooterEncoder;

  public ShooterSubsystem() {
    // Initializes motors using constants and configs
    preShooterMotor = new SparkMax(ShooterConstants.kPreShooterMotorCanId, MotorType.kBrushless);
    leftShooterMotor = new SparkFlex(ShooterConstants.kLeftShooterMotorCanId, MotorType.kBrushless);
    rightShooterMotor = new SparkFlex(ShooterConstants.kRightShooterMotorCanId, MotorType.kBrushless);

    preShooterMotor.configure(
      Configs.ShooterSubsystem.preShooterConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
    leftShooterMotor.configure(
      Configs.ShooterSubsystem.leftShooterConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
    rightShooterMotor.configure(
      Configs.ShooterSubsystem.rightShooterConfig,
      ResetMode.kResetSafeParameters,                                                   
      PersistMode.kPersistParameters);

    leftShooterEncoder = leftShooterMotor.getEncoder();
    rightShooterEncoder = rightShooterMotor.getEncoder();
  }

  public boolean shooterAtSpeed() { // Returns true if the motors are at full speed, doesn't have the right constants yet
    return 
      (leftShooterEncoder.getVelocity() == ShooterConstants.kLeftShooterMotorPower) &&
      (rightShooterEncoder.getVelocity() == ShooterConstants.kRightShooterMotorPower);
      
  }

  public Command runShooterCommand() { // Runs the main shooter
    return this.startEnd(
      () -> {
        preShooterMotor.set(ShooterConstants.kPreShooterMotorPower);
        leftShooterMotor.set(ShooterConstants.kLeftShooterMotorPower);
        rightShooterMotor.set(ShooterConstants.kRightShooterMotorPower);
      },
      () -> {
        preShooterMotor.set(0.0);
        leftShooterMotor.set(0.0);
        rightShooterMotor.set(0.0);
      }
    );
  }

  public Command idleShooterCommand() { // Runs the main shooter at idle power
    return this.run(
      () -> {
        leftShooterMotor.set(ShooterConstants.kLeftShooterIdlePower);
        rightShooterMotor.set(ShooterConstants.kRightShooterIdlePower);
      }
    );
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Velocity", leftShooterEncoder.getVelocity());
    SmartDashboard.putNumber("Right Velocity", rightShooterEncoder.getVelocity());
  }

}