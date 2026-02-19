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

  public enum Setpoint {
    kRetracted,
    kExtended
  }

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

    // pivotEncoder.setPosition(0);
    // pivotZeroed = true;

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

  public void tempZeroPivotAtIn() {
    pivotEncoder.setPosition(0.0);
    pivotTargetDeg = 0.0;
    pivotZeroed = true;
  }

  public void setPivotTargetDeg(double targetDeg) {
    // Clamp to your known safe range
    targetDeg = Math.max(0.0, Math.min(130.0, targetDeg));

    // Require zeroing first (until limit switch exists)
    if (!pivotZeroed) return;

    pivotTargetDeg = targetDeg;
    pivotController.setSetpoint(pivotTargetDeg,
      ControlType.kMAXMotionPositionControl);
  }

  public Command tempZeroPivotAtInCommand() {
    return this.runOnce(this::tempZeroPivotAtIn);
  }

  public Command pivotToDegCommand(double targetDeg) {
    return this.runOnce(() -> setPivotTargetDeg(targetDeg));
  }

  public Command pivotInCommand() {
    return pivotToDegCommand(0.0);
  }

  public Command pivotOutCommand() {
    return pivotToDegCommand(130.0);
  }

  // Only for testing
  public Command pivotJogCommand(double percent) {
    return this.startEnd(
      () -> pivotMotor.set(percent),
      () -> pivotMotor.set(0.0)
    );
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pivot Angle Deg", pivotEncoder.getPosition()); // Change erncoder units in Configs?
    SmartDashboard.putNumber("Pivot Target Deg", pivotTargetDeg);
    SmartDashboard.putBoolean("Pivot Zeroed", pivotZeroed);
  }
}