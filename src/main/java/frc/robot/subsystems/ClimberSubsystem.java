package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

    private SparkFlex climberMotor;

    private RelativeEncoder climberEncoder;

    private SparkClosedLoopController climberController;

    private double climberTargetDeg = 0;
    private boolean climberZeroed = false;

    public ClimberSubsystem() {
        climberMotor = new SparkFlex(ClimberConstants.kClimberMotorCanId, MotorType.kBrushless);

        climberMotor.configure(
            Configs.ClimberSubsystem.climberConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        climberEncoder = climberMotor.getEncoder();
        climberController = climberMotor.getClosedLoopController();

        climberEncoder.setPosition(0);
        climberZeroed = true;

    }

    /*----------Getters----------*/

    public boolean climberAtTarget() {
        double posErr = Math.abs(climberTargetDeg - climberEncoder.getPosition());
        double vel = Math.abs(climberEncoder.getVelocity());
        return posErr <= ClimberConstants.kClimberPosToleranceDeg
        && vel <= ClimberConstants.kClimberVelToleranceDegPerSec;
    }

    /*----------Setters----------*/

    public void setClimberTargetDeg(double targetDeg) {
        // Clamp to your known safe range
        targetDeg = Math.max(0.0, Math.min(50.0, targetDeg));

        // Require zeroing first
        if (!climberZeroed) return;
        
        climberTargetDeg = targetDeg;
        climberController.setSetpoint(
            climberTargetDeg,
            ControlType.kPosition // Maybe switch to maxMotion?
        );
    }

    /*----------Commands----------*/

    public Command climberToDegCommand(double targetDeg) { // Moves the climber to the given setpoint
        return this.runOnce(() -> setClimberTargetDeg(targetDeg))
            .andThen(run(() -> {}))
            .until(this::climberAtTarget);
    }

    public Command climberUpCommand() {
        return climberToDegCommand(50);
    }

    public Command climberDownCommand() {
        return climberToDegCommand(0);
    }

    /*----------Periodic----------*/

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber/Climber Pos", climberEncoder.getPosition());
        SmartDashboard.putNumber("Climber/Climber Target Pos", climberTargetDeg);
        SmartDashboard.putBoolean("Climber/Climber At Target", climberAtTarget());
    }
}
