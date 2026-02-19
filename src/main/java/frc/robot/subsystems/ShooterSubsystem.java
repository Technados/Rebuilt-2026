// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  private final SparkMax preShooterMotor;
  private final SparkFlex leftShooterMotor;
  private final SparkFlex rightShooterMotor;

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
  }

  public void setPower(SparkFlex motor, double power) { // Sets the power of the given SparkFlex motor
    motor.set(power);
  }

  public Command runShooterCommand() { // Runs the main shooter
    return this.startEnd(
      () -> {
        preShooterMotor.set(ShooterConstants.kPreShooterMotorPower);
        setPower(leftShooterMotor, ShooterConstants.kLeftShooterMotorPower);
        setPower(rightShooterMotor, ShooterConstants.kRightShooterMotorPower);
      },
      () -> {
        preShooterMotor.set(0.0);
        setPower(leftShooterMotor, 0.0);
        setPower(rightShooterMotor, 0.0);
      }
    );
  }

  public Command idleShooterCommand() { // Runs the main shooter at 20% power
    return this.run(
      () -> {
        setPower(leftShooterMotor, ShooterConstants.kLeftShooterIdlePower);
        setPower(rightShooterMotor, ShooterConstants.kRightShooterIdlePower);
      }
    );
  }

}