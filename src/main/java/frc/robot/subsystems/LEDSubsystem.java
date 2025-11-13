package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;

public class LEDSubsystem extends SubsystemBase {
    private final Spark blinkin;
    private double defaultPattern = -0.41; // any visual default

    private boolean overriding = false;
    private double overrideEndTime = 0;

    private boolean hasFlashedCoral = false;
    private boolean hasFlashedAlgae = false;
    private boolean hasFlashedGyroAlert = false;

    public LEDSubsystem(int pwmPort) {
        blinkin = new Spark(pwmPort);
        blinkin.set(defaultPattern);
    }

    public void setDefaultPattern(double pattern) {
        defaultPattern = pattern;
        if (!overriding) {
            blinkin.set(pattern);
        }
    }

    public void flashPattern(double pattern, double duration) {
        blinkin.set(pattern);
        overriding = true;
        overrideEndTime = Timer.getFPGATimestamp() + duration;
    }

    public void resetToDefault() {
        blinkin.set(defaultPattern);
        overriding = false;
    }

    @Override
    public void periodic() {
        if (overriding && Timer.getFPGATimestamp() >= overrideEndTime) {
            resetToDefault();
        }
    }

    // ==== Flash Once Helpers ====
    public void flashOnceForCoral(double pattern, double duration) {
        if (!hasFlashedCoral) {
            flashPattern(pattern, duration);
            hasFlashedCoral = true;
        }
    }

    public void flashOnceForAlgae(double pattern, double duration) {
        if (!hasFlashedAlgae) {
            flashPattern(pattern, duration);
            hasFlashedAlgae = true;
        }
    }

    public void flashOnceForGyroAlert(double pattern, double duration) {
        if (!hasFlashedGyroAlert) {
            flashPattern(pattern, duration);
            hasFlashedGyroAlert = true;
        }
    }

    public void clearCoralFlash() {
        hasFlashedCoral = false;
    }

    public void clearAlgaeFlash() {
        hasFlashedAlgae = false;
    }

    public void clearGyroAlertFlash() {
        hasFlashedGyroAlert = false;
    }
}
