// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Configs;
import frc.robot.Constants.IntakeConstants;

/*
 * Mechnical/Electrical Configuration:
 * 
 * Pivot Motor: 52.89
 * Intake: 1.5
 */

/**
 * {@link IntakeSubsystem}
 * 
 * <p>This subsystem is used to intake game pieces. It contains methods and commands for
 * the intake motor and pivot motor, as well as a periodic() method that publishes 
 * information to the dashboard.
 * 
 * <p>The intake motor is used to intake the game pieces, while the pivot 
 * motor is used to move the intake between setpoints. The pivot motor provides
 * feedback through its encoder that can be used to track its position. The intake
 * motor is power controller while the pivot motor is position controlled.
 */

public class IntakeSubsystem extends SubsystemBase {

  private final SparkFlex intakeMotor;
  private final SparkFlex pivotMotor;

  private final Servo hopperServo;

  private final DigitalInput zeroSwitch;

  private RelativeEncoder pivotEncoder;

  private SparkClosedLoopController pivotController;

  private boolean pivotZeroed = false;
  private double pivotTargetDeg;
  private double pivotIdlePower;

  public IntakeSubsystem() {
    // Initializes motors using constants and configs
    intakeMotor = new SparkFlex(IntakeConstants.kIntakeMotorCanId, MotorType.kBrushless);
    pivotMotor = new SparkFlex(IntakeConstants.kPivotMotorCanId, MotorType.kBrushless);
    
    hopperServo = new Servo(IntakeConstants.kHopperServoChannel);

    zeroSwitch = new DigitalInput(IntakeConstants.kZeroSwtichChannel);

    intakeMotor.configure(
      Configs.IntakeSubsystem.intakeConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );
    pivotMotor.configure(
      Configs.IntakeSubsystem.pivotConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );

    // Creates encoder to track the pivot motor's position
    pivotEncoder = pivotMotor.getEncoder();

    // Creates controller for pivot motor (intake uses power)
    pivotController = pivotMotor.getClosedLoopController();

    // Zeroes the encoder
    pivotEncoder.setPosition(0);
    pivotZeroed = true;

    pivotTargetDeg = 0.0;

    pivotIdlePower = IntakeConstants.kPivotIdleInPower;

    hopperServo.set(0); // Change position

  }

  /*----------Getters----------*/
  
  public double getPivotAngleDeg() {
    return pivotEncoder.getPosition();
  }

  public double getPivotVelocityDegPerSec() {
    return pivotEncoder.getVelocity();
  }

  public double getPivotCurrentAmps() {
    return pivotMotor.getOutputCurrent();
  }

  public boolean pivotLikelyDeadheaded() {
    double posErr = Math.abs(pivotTargetDeg - getPivotAngleDeg());
    double vel = Math.abs(getPivotVelocityDegPerSec());
    double current = getPivotCurrentAmps();

    return posErr > IntakeConstants.kPivotDeadheadPosErrDeg
        && vel < IntakeConstants.kPivotDeadheadMinVelDegPerSec
        && current > IntakeConstants.kPivotDeadheadCurrentAmps;
  }

  /**
   * Returns if the pivot is at the target postion.
   * @return Returns true if the pivot's position error and velocity are within the tolerance defined in the constants.
   */
  public boolean pivotAtTarget() { // Returns true if the pivot position is within the tolerance for the target
    double posErr = Math.abs(pivotTargetDeg - pivotEncoder.getPosition());
    return (posErr <= IntakeConstants.kPivotPosToleranceDeg);
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
  public void setPivotTargetDeg(double targetDeg) {
    // Clamp to known safe range
    targetDeg = Math.max(0.0, Math.min(100.0, targetDeg));

    // Require zeroing first
    if (!pivotZeroed) return;

    pivotTargetDeg = targetDeg;

    // Plain position control, not MAXMotion
    pivotController.setSetpoint(
      pivotTargetDeg,
      ControlType.kPosition,
      ClosedLoopSlot.kSlot0
  );
}

  public void zeroOnLimitPressed() {
    if (!zeroSwitch.get()) {
      tempZeroPivotAtIn();
      pivotIdlePower = IntakeConstants.kPivotIdleInPower;
    }
  }

  /*----------Commands----------*/
  
  /**
   * @return Command to run the intake motor.
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
   * @return Command to run the intake motor in reverse.
   */
  public Command runIntakeReverseCommand() { 
    return this.startEnd(
    () -> {
        intakeMotor.set(-IntakeConstants.kIntakeMotorPower);
      },
      () -> {
        intakeMotor.set(0.0);
      }
    );
  }

  public Command stopIntakeCommand() { 
    return this.runOnce(
      () -> {intakeMotor.set(0.0);}
    );
  }

  public Command runIntakeAgitateCommand() {
    return runOnce(
      () -> {intakeMotor.set(IntakeConstants.kIntakeMotorPower);}
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
    return Commands.sequence(
      this.runOnce(() -> {
        setPivotTargetDeg(targetDeg);
        if (targetDeg == 0) {
          pivotIdlePower = IntakeConstants.kPivotIdleInPower;
        } else {
          pivotIdlePower = IntakeConstants.kPivotIdleOutPower;
        }
      }),
      Commands.waitUntil(this::pivotAtTarget)
    );
  }

  private Command waitForPivotOrDeadhead(double timeoutSec) {
    return Commands.waitUntil(() -> pivotAtTarget() || pivotLikelyDeadheaded())
        .withTimeout(timeoutSec);
  }

  
  /**
   * @return Command to move pivot to 0 degrees.
   */
  public Command pivotInCommand() {
    return pivotToDegCommand(0.0);
  }
  
  /**
   * @return Command to move pivot to 100 degrees.
   */
  public Command pivotOutCommand() {
    return pivotToDegCommand(100.0);
  }


  /**
   * @return Command to run pivotAgitate, moves pivot to 0 degrees when the command ends.
   */
  public Command pivotAgitateCommand(BooleanSupplier ready) {
     return Commands.sequence(
      runIntakeAgitateCommand(),

      Commands.runOnce(() -> setPivotTargetDeg(IntakeConstants.kPivotAgitateJabDeg)),
      waitForPivotOrDeadhead(IntakeConstants.kPivotAgitateDownTimeoutSec),

      Commands.waitSeconds(IntakeConstants.kPivotAgitatePauseSec),

      Commands.runOnce(() -> setPivotTargetDeg(IntakeConstants.kPivotAgitateHomeDeg)),
      waitForPivotOrDeadhead(IntakeConstants.kPivotAgitateUpTimeoutSec),

      Commands.waitSeconds(IntakeConstants.kPivotAgitatePauseSec)
    )
      .onlyIf(ready)
      .repeatedly()
      .finallyDo(() -> {
        setPivotTargetDeg(IntakeConstants.kPivotAgitateHomeDeg);
        intakeMotor.set(0.0);
      });
  }

  public Command pivotIdleCommand() {
    return runEnd(
      () -> pivotMotor.set(pivotIdlePower),
      () -> pivotMotor.set(0)
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
    zeroOnLimitPressed();

    SmartDashboard.putNumber("Intake/Pivot Angle Deg", pivotEncoder.getPosition());
    SmartDashboard.putNumber("Intake/Pivot Target Deg", pivotTargetDeg);
    SmartDashboard.putBoolean("Intake/Pivot Zeroed", pivotZeroed);
    SmartDashboard.putNumber("Intake/Pivot Encoder Vel (degPerSec?)", pivotEncoder.getVelocity());
    SmartDashboard.putBoolean("Intake/Pivot At Target", pivotAtTarget());

    SmartDashboard.putNumber("Intake/Pivot Power", pivotMotor.getAppliedOutput());

    SmartDashboard.putBoolean("Intake/Limit Pressed", zeroSwitch.get());
  }

}