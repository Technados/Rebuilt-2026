package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;

/**
 * HoodSubsystem (PWM linear servos with modeled position)
 *
 * IMPORTANT:
 * These PWM linear actuators do NOT report true position back through the roboRIO
 * PWM ports. The servo object only knows what command value was sent, not where
 * the actuator physically ended up.
 *
 * Because of that, we use a simple time-based position model:
 * - targetPosition = commanded servo position
 * - currentPosition = estimated position that moves toward target over time
 *
 * Why do this?
 * - gives a stable "hood ready" signal for shooter sequencing
 * - matches the real behavior of a slow actuator better than assuming instant motion
 * - keeps command logic simple and understandable
 *
 * This is still NOT true closed-loop feedback.
 * If the mechanism binds, slips, gets bumped, or starts in the wrong place,
 * this model can still be wrong.
 *
 * Best future upgrade:
 * - read the actuator feedback potentiometer, or
 * - add a real hood angle sensor
 */
public class HoodSubsystem extends SubsystemBase {

    private final Servo leftServo;
    private final Servo rightServo;

    /**
     * Estimated hood position in normalized servo units.
     * This is a model, not a sensor reading.
     */
    private double currentPosition = HoodConstants.kStartingPosition;

    /**
     * Commanded hood target in normalized servo units.
     */
    private double targetPosition = HoodConstants.kStartingPosition;

    /**
     * Timestamp used for motion modeling.
     */
    private double lastUpdateTime;

    public HoodSubsystem() {
        leftServo = new Servo(HoodConstants.kLeftHoodServoPwm);
        rightServo = new Servo(HoodConstants.kRightHoodServoPwm);

        /**
         * Common PWM bounds used with these linear servos.
         * Better than relying only on generic servo defaults.
         */
        leftServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        rightServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);

        lastUpdateTime = Timer.getFPGATimestamp();

        /**
         * Start at the hood's normal "home" position.
         * This is the software's startup assumption.
         */
        setPosition(HoodConstants.kStartingPosition);
        currentPosition = HoodConstants.kStartingPosition;
    }

    /**
     * Command a hood position in normalized servo units.
     * The value is clamped to a reduced safe/useful range for match play.
     */
    public void setPosition(double position) {
        double clampedPosition = MathUtil.clamp(
            position,
            HoodConstants.kMinPos,
            HoodConstants.kMaxPos
        );

        leftServo.set(clampedPosition);
        rightServo.set(clampedPosition);
        targetPosition = clampedPosition;
    }

    /**
     * Sends the hood back to its default "home" position.
     * This is the close-shot / auto-start / normal-rest position.
     */
    public void home() {
        setPosition(HoodConstants.kStartingPosition);
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public double getCurrentPosition() {
        return currentPosition;
    }

    /**
     * Returns true when the modeled position is within tolerance of target.
     * This is what the ready-to-feed logic should use for hood readiness.
     */
    public boolean atTarget() {
        return MathUtil.isNear(
            targetPosition,
            currentPosition,
            HoodConstants.kPosTolerance
        );
    }

    /**
     * One-shot move command:
     * - command hood to a position once
     * - wait until the modeled position reaches tolerance
     *
     * Useful for autonomous or fixed-position sequencing.
     */
    public Command positionCommand(double position) {
        return runOnce(() -> setPosition(position))
            .andThen(Commands.waitUntil(this::atTarget));
    }

    /**
     * Continuously command a fixed hood position while the command is active.
     * Useful for live shot prep commands that stay scheduled while a trigger is held.
     */
    public Command holdPositionCommand(java.util.function.DoubleSupplier positionSupplier) {
        return run(() -> setPosition(positionSupplier.getAsDouble()));
    }

    /**
     * Continuously command the home position while active.
     */
    public Command holdHomeCommand() {
        return run(this::home);
    }

    /**
     * One-shot command to move back to home and wait until the modeled hood gets there.
     */
    public Command homeCommand() {
        return positionCommand(HoodConstants.kStartingPosition);
    }

    /**
     * Jog command for testing.
     * Uses targetPosition so repeated taps behave consistently.
     */
    public Command jogCommand(double delta) {
        return runOnce(() -> setPosition(targetPosition + delta));
    }

    /**
     * Move the modeled position toward the target based on actuator speed.
     *
     * 150:1 / 100 mm actuator:
     * - theoretical max no-load speed is about 8 mm/sec
     * - normalized theoretical max is about 8/100 = 0.08 units/sec
     *
     * We use a slightly reduced modeled speed so "ready" does not become true
     * too early under real robot conditions.
     */
    private void updateModeledPosition() {
        double now = Timer.getFPGATimestamp();
        double dt = now - lastUpdateTime;
        lastUpdateTime = now;

        double maxStep = HoodConstants.kModeledPosUnitsPerSec * dt;
        double error = targetPosition - currentPosition;

        if (Math.abs(error) <= maxStep) {
            currentPosition = targetPosition;
        } else {
            currentPosition += Math.copySign(maxStep, error);
        }
    }

    @Override
    public void periodic() {
        updateModeledPosition();

        SmartDashboard.putNumber("Hood/TargetPosition", targetPosition);
        SmartDashboard.putNumber("Hood/CurrentPositionModeled", currentPosition);
        SmartDashboard.putBoolean("Hood/AtTarget", atTarget());
    }
}