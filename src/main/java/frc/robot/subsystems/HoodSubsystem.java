package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;

/**
* HoodSubsystem (PWM Servos)
*
* PWM servos generally do NOT provide position feedback to the roboRIO.
* That means we cannot truly "measure" when the hood arrives.
*
* For sequencing ("don't shoot until hood is set"), we use a simple modeled position
* that moves toward the target at a limited speed. It's not perfect physics, but it
* creates a stable, teachable definition of "at target".
*
* Later upgrade option:
* - Add a real sensor (pot/encoder) for true closed-loop hood angle control.
*/

public class HoodSubsystem extends SubsystemBase {

    private final Servo leftServo;
    private final Servo rightServo;

    private double targetPos = 0;
    private double modeledPos = 0;

    private double lastTime;

    public HoodSubsystem() {
        // Initializes servos
        leftServo = new Servo(HoodConstants.kLeftHoodServoPwm);
        rightServo = new Servo(HoodConstants.kRightHoodServoPwm);

        // Sets target and modeled positions
        modeledPos = 0.3;

        // Sets timestamp
        lastTime = Timer.getFPGATimestamp();

        // Sets hood position to the target position
        setHoodPosition(0.3);

        targetPos = 0.3;
    }

    /*----------Setters----------*/

    /**
     * Sets the hood position.
     * @param pos The target position
     */
    public void setHoodPosition(double pos) {
        targetPos = MathUtil.clamp(pos, HoodConstants.kMinPos, HoodConstants.kMaxPos);
        leftServo.set(targetPos);
        rightServo.set(targetPos);
    }

    /*----------Getters----------*/

    /**
     * @return The current target position
     */
    public double getTargetPos() {
        return targetPos;
    }

    /**
     * Returns true if postion is within target tolerance.
     * @return Boolean representing if position is at target
     */
    public boolean atTarget() {
        return Math.abs(modeledPos - targetPos) <= HoodConstants.kPosTolerance;
    }

    /*----------Commands----------*/

    /**
     * Returns hood jog command. This command is only for testing.
     * @param posIncrease The position increment that the servos will move at.
     * @return Command to move the hood manually.
     */
    public Command hoodJogCommand(double posIncrease) {
        return this.startEnd(
            () -> setHoodPosition(modeledPos + posIncrease),
            () -> holdPositionCommand(modeledPos)
        );
    }

    /**
     * Holds a hood position continuously 
     * (useful for ReadyToShoot)
     * @param posSupplier Target position for hood
     * @return Command to set and hold hood position
     */
    public Command holdPositionCommand(DoubleSupplier posSupplier) {
        return run(() -> setHoodPosition(posSupplier.getAsDouble()));
    }

    /**
     * Hold a hood position continuously 
     * (useful for ReadyToShoot)
     * @param position Target position for hood
     * @return Command to set and hold hood position
     */
    public Command holdPositionCommand(double position) {
        return run(() -> setHoodPosition(position));
    }

    /*----------Periodic----------*/

    @Override
    public void periodic() {
        double now = Timer.getFPGATimestamp();
        double dt = now - lastTime;
        lastTime = now;

        // model movement for a reliable "atTarget" signal
        double maxStep = HoodConstants.kMaxPosUnitsPerSec * dt;
        double error = targetPos - modeledPos;

        if (Math.abs(error) <= maxStep) modeledPos = targetPos;
        else modeledPos += Math.copySign(maxStep, error);

        SmartDashboard.putNumber("Hood/TargetPos", targetPos);
        SmartDashboard.putNumber("Hood/ModeledPos", modeledPos);
        SmartDashboard.putBoolean("Hood/AtTarget", atTarget());
    }

}
