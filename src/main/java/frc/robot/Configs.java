package frc.robot;

import com.revrobotics.spark.FeedbackSensor;
//import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VortexConstants;

public final class Configs {
  public static final class MAXSwerveModule {
    public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

    static {

      // Use module constants to calculate conversion factors and feed forward gain.
      double drivingFactor = 
          ModuleConstants.kWheelDiameterMeters * Math.PI / ModuleConstants.kDrivingMotorReduction;
      double turningFactor = 2 * Math.PI;
      double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

      drivingConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50);
      drivingConfig
          .encoder
          .positionConversionFactor(drivingFactor) // meters
          .velocityConversionFactor(drivingFactor / 60.0); // meters per second
      
      drivingConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // These are example gains you may need to them for your own robot!
          .pid(0.04, 0, 0.003)
          .outputRange(-1, 1)
          .feedForward
            .kV(drivingVelocityFeedForward);

      turningConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20);
      turningConfig
          .absoluteEncoder
          // Invert the turning encoder, since the output shaft rotates in the opposite
          // direction of the steering motor in the MAXSwerve Module.
          .inverted(true)
          .positionConversionFactor(turningFactor) // radians
          .velocityConversionFactor(turningFactor / 60.0); // radians per second
      turningConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          // These are example gains you may need to them for your own robot!
          .pid(1, 0, 0)
          .outputRange(-1, 1)
          // Enable PID wrap around for the turning motor. This will allow the PID
          // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
          // to 10 degrees will go through 0 rather than the other direction which is a
          // longer route.
          .positionWrappingEnabled(true)
          .positionWrappingInputRange(0, turningFactor);
    }
  }

  public static final class ClimberSubsystem {
    public static final SparkFlexConfig climberConfig = new SparkFlexConfig();

    static {
      climberConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(40);

      climberConfig.encoder.positionConversionFactor(ClimberConstants.kClimberDegPerMotorRev);   // deg per motor rev;
      climberConfig.encoder.velocityConversionFactor(ClimberConstants.kClimberDegPerSecPerMotorRPM); // deg/sec per motor RPM

      climberConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ClimberConstants.kClimberP, ClimberConstants.kClimberI, ClimberConstants.kClimberD)
        .outputRange(-0.35, 0.35);
    }
  }

  public static final class IntakeSubsystem {
    public static final SparkFlexConfig intakeConfig = new SparkFlexConfig();
    public static final SparkFlexConfig pivotConfig = new SparkFlexConfig();

    static {
      // Intake Configs
      intakeConfig.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(40);

      // Pivot Configs
      pivotConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(40);

      pivotConfig.encoder.positionConversionFactor(IntakeConstants.kPivotDegPerMotorRev);   // deg per motor rev;
      pivotConfig.encoder.velocityConversionFactor(IntakeConstants.kPivotDegPerSecPerMotorRPM); // deg/sec per motor RPM

      pivotConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(IntakeConstants.kPivotP, IntakeConstants.kPivotI, IntakeConstants.kPivotD)
        .outputRange(-0.35, 0.35);

      pivotConfig.closedLoop.maxMotion
        .cruiseVelocity(IntakeConstants.kPivotCruiseVelocity)
        .maxAcceleration(IntakeConstants.kPivotMaxAccel);

    }
  }

  public static final class FeederSubsystem {
    public static final SparkFlexConfig feederConfig = new SparkFlexConfig();
    public static final SparkMaxConfig preShooterConfig = new SparkMaxConfig();
    
    static {
      // Configure basic settings of the feeder and preshooter motors
      feederConfig.inverted(true).idleMode(IdleMode.kCoast).smartCurrentLimit(40);
      preShooterConfig.inverted(false).idleMode(IdleMode.kCoast).smartCurrentLimit(40);
    }
  }
  
  public static final class ShooterSubsystem {
    public static final SparkFlexConfig leftShooterConfig = new SparkFlexConfig();
    public static final SparkFlexConfig rightShooterConfig = new SparkFlexConfig();
    
    static {
      // Right Shooter Configs
      rightShooterConfig
      .inverted(true)
      .idleMode(IdleMode.kCoast)
      .closedLoopRampRate(1.0)
      .openLoopRampRate(1.0)
      .smartCurrentLimit(40)
      .voltageCompensation(12.0);
      
      rightShooterConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(ShooterConstants.kShooterP, ShooterConstants.kShooterI, ShooterConstants.kShooterD)
      .outputRange(-1.0, 1.0);
      
      rightShooterConfig.closedLoop.feedForward
      .kV(1.0 / VortexConstants.kFreeSpeedRpm);
      
      // Left Shooter Configs
      leftShooterConfig.apply(rightShooterConfig)
      .follow(ShooterConstants.kRightShooterMotorCanId, true);
      
    }
  }

}
