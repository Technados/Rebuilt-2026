package frc.robot.subsystems;

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

    private final Servo leftServo = new Servo(HoodConstants.kLeftHoodServoPwm);
    private final Servo rightServo = new Servo(HoodConstants.kRightHoodServoPwm);

    private double targetPos = 0.50;
    private double modeledPos = 0.50;

    private double lastTime = Timer.getFPGATimestamp();

    public HoodSubsystem() {
        setHoodPosition(targetPos);
    }

    public void setHoodPosition(double pos) {
        targetPos = clamp(pos, HoodConstants.kMinPos, HoodConstants.kMaxPos);
        leftServo.set(targetPos);
        rightServo.set(targetPos);
    }

    public double getTargetPos() {
        return targetPos;
    }

    public boolean atTarget() {
        return Math.abs(modeledPos - targetPos) <= HoodConstants.kPosTolerance;
    }

    /** Hold a hood position continuously (useful for ReadyToShoot). */
    public Command holdPositionCommand(java.util.function.DoubleSupplier posSupplier) {
        return run(() -> setHoodPosition(posSupplier.getAsDouble()));
    }

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

    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }


}
