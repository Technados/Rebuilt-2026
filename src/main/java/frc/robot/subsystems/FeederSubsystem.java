package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.FeederConstants;

/*
 * Mechnical/Electrical Configuration:
 * 
 * Feeder: 6
 */

public class FeederSubsystem extends SubsystemBase {

    private final SparkFlex feederMotor;
    
    public FeederSubsystem() {
        // Initializes motor using constants and configs
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

    public Command runFeederCommand() { // Runs the feeder motor
        return this.startEnd(
      () -> {
        setPower(feederMotor, FeederConstants.kFeederMotorPower);
      },
      () -> {
        setPower(feederMotor, 0.0);
      }
    );
    }

}
