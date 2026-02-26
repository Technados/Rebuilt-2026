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
  private SparkClosedLoopController pivotController;
  private boolean pivotZeroed = false;
  private double pivotTargetDeg = 0.0;

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

    // Tracks the pivot motor's position
    pivotEncoder = pivotMotor.getEncoder();

    pivotController = pivotMotor.getClosedLoopController();

    pivotEncoder.setPosition(0);
    pivotZeroed = true;

    System.out.println("Intake initialized");

  }

  /* Intake */

  public Command runIntakeCommand() { // Runs the intake
    return this.startEnd(
    () -> {
        intakeMotor.set(IntakeConstants.kIntakeMotorPower);
      },
      () -> {
        intakeMotor.set(0.0);
      }
    );
  }

  /* Pivot */

  public boolean pivotAtTarget() {
    double posErr = Math.abs(pivotTargetDeg - pivotEncoder.getPosition());
    double vel = Math.abs(pivotEncoder.getVelocity());
    return posErr <= IntakeConstants.kPivotPosToleranceDeg
      && vel <= IntakeConstants.kPivotVelToleranceDegPerSec;
  }

  public void tempZeroPivotAtIn() { // Sets zero to the current pivot position
    pivotEncoder.setPosition(0.0);
    pivotTargetDeg = 0.0;
    pivotZeroed = true;
  }

  public void setPivotTargetDeg(double targetDeg) { // Sets the pivot target
    // Clamp to your known safe range
    targetDeg = Math.max(0.0, Math.min(120.0, targetDeg));

    // Require zeroing first (until limit switch exists)
    if (!pivotZeroed) return;

    pivotTargetDeg = targetDeg;
    pivotController.setReference(pivotTargetDeg,
      ControlType.kMAXMotionPositionControl);
  }

  public Command tempZeroPivotAtInCommand() { // Returns tempZeroPivotAtIn as a command
    return this.runOnce(this::tempZeroPivotAtIn);
  }

  public Command pivotToDegCommand(double targetDeg) { // Moves the pivot to the given setpoint
    return this.runOnce(() -> setPivotTargetDeg(targetDeg));
  }

  public Command pivotInCommand() { // Moves the pivot to 80
    return pivotToDegCommand(80.0)
      .until(this::pivotAtTarget);
  }

  public Command pivotOutCommand() { // Moves the pivot to 0
    return pivotToDegCommand(0.0)
      .until(this::pivotAtTarget);
  }

  // Only for testing
  public Command pivotJogCommand(double percent) { // Moves the pivot manually
    return this.startEnd(
      () -> pivotMotor.set(percent),
      () -> pivotMotor.set(0.0)
    );
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake/Pivot Angle Deg", pivotEncoder.getPosition());
    SmartDashboard.putNumber("Intake/Pivot Target Deg", pivotTargetDeg);
    SmartDashboard.putBoolean("Intake/Pivot Zeroed", pivotZeroed);
    SmartDashboard.putNumber("Intake/Pivot Encoder Pos (deg)", pivotEncoder.getPosition());
    SmartDashboard.putNumber("Intake/Pivot Encoder Vel (degPerSec?)", pivotEncoder.getVelocity());
  }
}