package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Timer;
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
    // Initializes motors using constants and configs
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

  /*----------Control Methods----------*/

  public void setPower(double feederPower, double preShooterPower) { // Sets the power of both motors
    feederMotor.set(feederPower);
    preShooterMotor.set(preShooterPower);
  }

  public void stop() { // Stops both motors
    setPower(0, 0);
  }

  /*----------Commands----------*/

  public Command runFeederCommand(ShooterSubsystem shooter) { // Runs the feeder while command is active
    return this.startEnd(
      () -> setPower(FeederConstants.kFeederMotorPower, FeederConstants.kPreShooterMotorPower),
      this::stop
    );
  }

  /* Feeds ONLY when enabledSupplier is true
    when aimed + at RPM + hood ready */
  public Command feedWhen(BooleanSupplier enabledSupplier) {
    return this.run(() -> {
      if (enabledSupplier.getAsBoolean()) {
        setPower(FeederConstants.kFeederMotorPower, FeederConstants.kPreShooterMotorPower);
      } else {
        stop();
      }
    });
  }

  public Command feedWhen(boolean shooterAtSpeed) { // Feeds only when shooterAtSpeed is true
    return this.run(() -> {
      if (shooterAtSpeed) {
        setPower(FeederConstants.kFeederMotorPower, FeederConstants.kPreShooterMotorPower);
      } else {
        stop();
      }
    });
  }
 
  public Command pulseFeed(double seconds) { // Pulses the feeder for a short duration (useful for single-shot testing)
    return this.runOnce(() -> setPower(FeederConstants.kFeederMotorPower, FeederConstants.kPreShooterMotorPower))
      .andThen(this.waitSeconds(seconds))
      .andThen(this.runOnce(this::stop));
  }

  private Command waitSeconds(double seconds) { // Small helper (keeps timing logic out of RobotContainer)
    return this.run(() -> {}).until(new BooleanSupplier() {
      final double start = Timer.getFPGATimestamp();
      @Override public boolean getAsBoolean() {
        return Timer.getFPGATimestamp() - start >= seconds;
      }
    });
  }

}
