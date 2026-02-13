// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
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

  public IntakeSubsystem() {
    intakeMotor = new SparkFlex(IntakeConstants.kIntakeMotorCanId, MotorType.kBrushless);

    intakeMotor.configure(
      Configs.IntakeSubsystem.intakeConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
  }

  public Command runIntakeCommand() {
    return this.startEnd(
      () -> {
        setPower(IntakeConstants.kIntakePower);
      },
      () -> {
        setPower(0.0);
      }
    );
  }

  public void setPower(double power) {
    intakeMotor.set(power);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Current", intakeMotor.getOutputCurrent());
  }
}