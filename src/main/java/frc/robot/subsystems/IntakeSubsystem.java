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
      PersistMode.kPersistParameters
    );
    pivotMotor.configure(
      Configs.IntakeSubsystem.pivotConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kNoPersistParameters
    );

    // Creates encoder to track the pivot motor's position
    pivotEncoder = pivotMotor.getEncoder();

    // Creates controller for pivot motor (intake uses power)
    pivotController = pivotMotor.getClosedLoopController();

    // Zeroes the encoder
    pivotEncoder.setPosition(0);
    pivotZeroed = true;

  }

  /*----------Getters----------*/
  
  /**
   * Returns if the pivot is at the target postion.
   * @return Returns true if the pivot's position error and velocity are within the tolerance defined in the constants.
   */
  public boolean pivotAtTarget() { // Returns true if the pivot position is within the tolerance for the target
    double posErr = Math.abs(pivotTargetDeg - pivotEncoder.getPosition());
    double vel = Math.abs(pivotEncoder.getVelocity());
    return posErr <= IntakeConstants.kPivotPosToleranceDeg
    && vel <= IntakeConstants.kPivotVelToleranceDegPerSec;
  }

  /*----------Control Methods----------*/
  
  /**
   * Sets the pivot's current position to zero.
   */
  public void tempZeroPivotAtIn() {
    pivotEncoder.setPosition(0.0);
    pivotTargetDeg = 0.0;
    pivotZeroed = true;
  }
  
  /**
   * Sets the pivot's postion to the target position.
   * @param targetDeg The pivot's target position in degrees.
   */
  public void setPivotTargetDeg(double targetDeg) { // Sets the pivot target
    // Clamp to your known safe range
    targetDeg = Math.max(0.0, Math.min(100.0, targetDeg));
    
    // Require zeroing first (until limit switch exists)
    if (!pivotZeroed) return;
    
    pivotTargetDeg = targetDeg;
    pivotController.setSetpoint(
      pivotTargetDeg,
      ControlType.kMAXMotionPositionControl
    );
  }

  /**
   * Moves the pivot to 0 degrees, then 70. Can be used in a command to 
   * move the pivot back and forth continuously.
   */
  public void pivotAgitate() {
    setPivotTargetDeg(0);
    setPivotTargetDeg(70);
  }

  /*----------Commands----------*/
  
  /**
   * @return Command to run the intake mator.
   */
  public Command runIntakeCommand() { 
    return this.startEnd(
    () -> {
        intakeMotor.set(IntakeConstants.kIntakeMotorPower);
      },
      () -> {
        intakeMotor.set(0.0);
      }
    );
  }

  /**
   * Returns tempZeroPivotAtIn as a command.
   */
  public Command tempZeroPivotAtInCommand() {
    return this.runOnce(this::tempZeroPivotAtIn);
  }
  
  /**
   * Moves the pivot to the the target position.
   * @param targetDeg The target postion in degrees.
   * @return Command to move the pivot motor to the target position.
   */
  public Command pivotToDegCommand(double targetDeg) {
    return this.runOnce(() -> setPivotTargetDeg(targetDeg))
      .andThen(run(() -> {}))
      .until(this::pivotAtTarget);
  }
  
  /**
   * @return Command to move pivot to 100 degrees.
   */
  public Command pivotInCommand() {
    return pivotToDegCommand(100.0);
  }
  
  /**
   * @return Command to move pivot to 0 degrees.
   */
  public Command pivotOutCommand() {
    return pivotToDegCommand(0.0);
  }

  /**
   * @return Command to run pivotAgitate, moves pivot to 0 degrees when the command ends.
   */
  public Command pivotAgitateCommand() {
    return runEnd(
      () -> pivotAgitate(),
      () -> setPivotTargetDeg(0)
    );
  }
  
  /**
   * Returns pivot jog command. This command is only for testing.
   * @param percent The speed the pivot will move at.
   * @return Command to move the pivot manually.
   */
  public Command pivotJogCommand(double percent) {
    return this.startEnd(
      () -> pivotMotor.set(percent),
      () -> pivotMotor.set(0.0)
    );
  }

  /*----------Periodic----------*/

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake/Pivot Angle Deg", pivotEncoder.getPosition());
    SmartDashboard.putNumber("Intake/Pivot Target Deg", pivotTargetDeg);
    SmartDashboard.putBoolean("Intake/Pivot Zeroed", pivotZeroed);
    SmartDashboard.putNumber("Intake/Pivot Encoder Vel (degPerSec?)", pivotEncoder.getVelocity());
    SmartDashboard.putBoolean("Intake/Pivot At Target", pivotAtTarget());
  }
}