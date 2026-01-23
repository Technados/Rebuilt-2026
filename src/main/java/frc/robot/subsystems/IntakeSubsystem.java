// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  /** Creates a new Intake. */
  private static IntakeSubsystem instance;

  // Motor Controllers
  private SparkFlex intakeMotor;

  // Motor Configs
  private SparkFlexConfig config;

  // Relative Encoders
  private RelativeEncoder intakeEncoder;

  // Stores the speed of the intake motor
  private float intakeSpeed;

  public IntakeSubsystem() {

    intakeMotor = new SparkFlex(Constants.IntakeConstants.kIntakeMotorCanId, MotorType.kBrushless);
    config = new SparkFlexConfig();
    intakeEncoder = intakeMotor.getEncoder();
    intakeSpeed = Constants.IntakeConstants.kIntakeSpeed;

    // Reset the motors
    //intakeMotor.restoreFactoryDefaults();

    config
        .inverted(false)
        .idleMode(IntakeConstants.kIntakeIdleMode);

    intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Resetting the encoder postion on robot startup
    resetEncoders();

  }

   // Returns an instance of this subsystem
  public static IntakeSubsystem getInstance() {
    if (instance == null) {
      instance = new IntakeSubsystem();
    }
    return instance;
  }

  // Spins the intake motors forwards
  public void startIntake() {
    intakeMotor.set(intakeSpeed);
  }

  // Spins the intake motors reverse
  public void reverseIntake() {
    intakeMotor.set(-intakeSpeed);
  }

  // Stops the intake motors
  public void stopIntake() {
    intakeMotor.stopMotor();
  }

  // Resets the position of the intake encoder to 0.0
  public void resetEncoders() {
    intakeEncoder.setPosition(0);
  }

  // Returns the encoder position
  public double getEncoderPosition() {
    return intakeEncoder.getPosition();
  }

  // Auto method for intaking a game piece
  public boolean autoIntake() {
    resetEncoders();
    while (getEncoderPosition() <= 50.0) {
      // value of 50.0 is arbitrary -- will test how long shooter runs for with this
      // value and change as needed
      startIntake();
    }
    stopIntake();
    return true;
  }
}