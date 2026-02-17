package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.IntakeConstants;

/*
 * Mechnical/Electrical Configuration:
 * 
 * Feeder: 6
 */

public class FeederSubsystem extends SubsystemBase {

    private final SparkFlex feederMotor;
    
    public FeederSubsystem() {
        feederMotor = new SparkFlex(FeederConstants.kFeederMotorCanId, MotorType.kBrushless);

        feederMotor.configure(
            Configs.IntakeSubsystem.intakeConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    public void setPower(SparkFlex motor, double power) { // Sets the power of the given motor
        motor.set(power);
    }

    public Command runFeederCommand() {
        return this.startEnd(
      () -> {
        setPower(feederMotor, IntakeConstants.kIntakeMotorPower);
      },
      () -> {
        setPower(feederMotor, 0.0);
      }
    );
    }

}
