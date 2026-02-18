// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.IntakeConstants;

/*
 * Mechnical/Electrical Configuration:
 * 
 * Pivot Motor: 52.89
 * Intake: 1.5
 */

public class IntakeSubsystem extends SubsystemBase {

  private final SparkFlex intakeMotor;
  private final SparkFlex pivotMotor;

  private RelativeEncoder pivotEncoder;

  public IntakeSubsystem() {
    // Initializes motors using constants and configs
    intakeMotor = new SparkFlex(IntakeConstants.kIntakeMotorCanId, MotorType.kBrushless);
    pivotMotor = new SparkFlex(IntakeConstants.kPivotMotorCanId, MotorType.kBrushless);

    intakeMotor.configure(
      Configs.IntakeSubsystem.intakeConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
    pivotMotor.configure(
      Configs.IntakeSubsystem.pivotConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kNoPersistParameters
    );

    // Tracks the pivot motor's position, sets to 0 when intitialized
    pivotEncoder = pivotMotor.getEncoder();
    pivotEncoder.setPosition(0);
  }
  
  public void setPower(SparkFlex motor, double power) { // Sets the power of the given motor
    motor.set(power);
  }

  public Command runIntakeCommand() { // Runs the intake
    return this.startEnd(
    () -> {
        setPower(intakeMotor, IntakeConstants.kIntakeMotorPower);
      },
      () -> {
        setPower(intakeMotor, 0.0);
      }
    );
  }

  public Command runPivotForwardCommand() { // Runs the pivot motor forward
    return this.startEnd(
      () -> {
        setPower(pivotMotor, IntakeConstants.kPivotMotorPower);
      },
      () -> {
        setPower(pivotMotor, 0.0);
      }
    );
  }

  public Command runPivotReverseCommand() { // Runs the pivot motor in reverse
    return this.startEnd(
      () -> {
        setPower(pivotMotor, -IntakeConstants.kPivotMotorPower);
      },
      () -> {
        setPower(pivotMotor, 0.0);
      }
    );
  }

  public Command resetRetractedPivotCommand() { // Resets the pivot motor to a retracted position
    return this.run(
      () -> {
        if (pivotEncoder.getPosition() > IntakeConstants.kPivotMotorRetracted) {
          runPivotReverseCommand();
        }
      }
    );
  }

  public Command resetExtendedPivotCommand() { // Resets the pivot motor to an extended position
    return this.run(
      () -> {
        if (pivotEncoder.getPosition() < IntakeConstants.kPivotMotorExtended) {
          runPivotForwardCommand();
        }
      }
    );
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Current", intakeMotor.getOutputCurrent());
    SmartDashboard.putNumber("Pivot Angle", pivotEncoder.getPosition());
  }
}