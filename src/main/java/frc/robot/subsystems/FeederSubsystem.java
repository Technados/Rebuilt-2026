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

  /**
   * Runs the motors at their power defined in the constants.
   */
  public void setPower() {
    feederMotor.set(FeederConstants.kFeederMotorPower);
    preShooterMotor.set(FeederConstants.kPreShooterMotorPower);
  }

  /**
   * Stops the motors.
   */
  public void stop() { 
    feederMotor.set(0);
    preShooterMotor.set(0);
  }

  /*----------Commands----------*/

  /**
   * @return Command to run the feeder motor.
   */
  public Command runFeederCommand() {
    return this.startEnd(
      () -> setPower(),
      this::stop
    );
  }

  /**
   * Feeds only when enabledSupplier is true
   * (when aimed + at RPM + hood ready).
   * @param enabledSupplier Boolean representing if the shooter is up to speed.
   * @return Command to conditionally run the feeder.
   */
  
  public Command feedWhen(BooleanSupplier enabledSupplier) {
    return this.runEnd(
      () -> {
        boolean ready = enabledSupplier.getAsBoolean();

        if (ready) {
          setPower();
        }
      },
      () -> {
        this.stop();
      }
    );
  }
 
  /**
   * Pulses the feeder for a short duration 
   * (useful for single-shot testing)
   * @param seconds How long the command will run for.
   * @return Command to pulse the feeder.
   */
  public Command pulseFeed(double seconds) {
    return this.runOnce(() -> setPower())
      .andThen(this.waitSeconds(seconds))
      .andThen(this.runOnce(this::stop));
  }

  /**
   * Delays a command sequence 
   * (keeps timing logic out of RobotContainer)
   * @param seconds How long the command will delay for.
   * @return Command to delay for a parmeterized amount of time.
   */
  private Command waitSeconds(double seconds) {
    return this.run(() -> {}).until(new BooleanSupplier() {
      final double start = Timer.getFPGATimestamp();
      @Override public boolean getAsBoolean() {
        return Timer.getFPGATimestamp() - start >= seconds;
      }
    });
  }

}
