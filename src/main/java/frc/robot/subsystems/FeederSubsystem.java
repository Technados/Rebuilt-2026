package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

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
    private final SparkMax preShooterMotor;
    
    public FeederSubsystem() {
        // Initializes motor using constants and configs
        feederMotor = new SparkFlex(FeederConstants.kFeederMotorCanId, MotorType.kBrushless);
        preShooterMotor = new SparkMax(FeederConstants.kPreShooterMotorCanId, MotorType.kBrushless);

        feederMotor.configure(
            Configs.FeederSubsystem.feederConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
        preShooterMotor.configure(
            Configs.FeederSubsystem.preShooterConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    public Command runFeederCommand(ShooterSubsystem shooter) { // Runs the feeder motor once the shooter is up to speed
      return this.startEnd(
        () -> {
          while (!shooter.shooterAtVelocity()) {
            continue;
          }

          feederMotor.set(FeederConstants.kFeederMotorPower);
          preShooterMotor.set(FeederConstants.kPreShooterMotorPower);
        },
        () -> {
          feederMotor.set(0.0);
          preShooterMotor.set(0.0);
        }
      );
    }

}
