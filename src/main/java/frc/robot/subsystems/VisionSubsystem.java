package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionConstants;
import frc.robot.vision.VisionMeasurement;

public class VisionSubsystem extends SubsystemBase {

    private VisionMeasurement visionMeasurementFront;
    private VisionMeasurement visionMeasurementBack;

    private String frontLimelightName;
    private String backLimelightName;

    public VisionSubsystem() {
        frontLimelightName = Constants.VisionConstants.kFrontLimelightName;
        backLimelightName = Constants.VisionConstants.kBackLimelightName;

        visionMeasurementFront = new VisionMeasurement(LimelightHelpers.getBotPose2d(frontLimelightName), 0.0, Constants.VisionConstants.kMultiTagStdDevs);
        visionMeasurementBack = new VisionMeasurement(LimelightHelpers.getBotPose2d(backLimelightName), 0.0, Constants.VisionConstants.kMultiTagStdDevs);

        SmartDashboard.putBoolean("Vision valid?", hasValidPoseFront());
        SmartDashboard.putNumberArray("Vision Pose x/y/rot", visionMeasurementFront.getStdDevsList()); //Add actual values from VisionMeasurement
        SmartDashboard.putNumber("Tag count", getTagCountFront());
        SmartDashboard.putNumber("Latency", getLatencyFrontSeconds());
    }

    public boolean hasValidPoseFront() {
        return LimelightHelpers.validPoseEstimate(LimelightHelpers.getBotPoseEstimate_wpiBlue(frontLimelightName));
    }

    public int getTagCountFront() {
        return LimelightHelpers.getTargetCount(frontLimelightName);
    }

    public double getLatencyFrontSeconds() {
        return LimelightHelpers.getLatency_Pipeline(frontLimelightName);
    }

    public Optional<VisionMeasurement> getVisionMeasurementFront() {

        if (!LimelightHelpers.getTV(frontLimelightName)) {
            return Optional.empty();
        }

        Pose2d visionPose = LimelightHelpers.getBotPose2d_wpiBlue(frontLimelightName);
        double latencyMs = LimelightHelpers.getLatency_Capture(frontLimelightName) + LimelightHelpers.getLatency_Pipeline(frontLimelightName);
        double timestampSeconds = Timer.getFPGATimestamp() - (latencyMs / 1000.0);
        int tagCount = LimelightHelpers.getTargetCount(frontLimelightName);

        Matrix<N3, N1> stdDevs = 
            tagCount >= 2
                ?VisionConstants.kMultiTagStdDevs
                :VisionConstants.kSingleTagStdDevs;

        VisionMeasurement measurement = new VisionMeasurement(visionPose, timestampSeconds, stdDevs);

        return Optional.of(measurement);

    }
}