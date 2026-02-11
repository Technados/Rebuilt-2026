package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionConstants;
import frc.robot.vision.VisionMeasurement;

public class VisionSubsystem extends SubsystemBase {

    public boolean hasValidPose(String name) {
        return LimelightHelpers.validPoseEstimate(LimelightHelpers.getBotPoseEstimate_wpiBlue(name));
    }

    public int getTagCount(String name) {
        return LimelightHelpers.getTargetCount(name);
    }

    public double getLatencySeconds(String name) {
        return LimelightHelpers.getLatency_Capture(name) + LimelightHelpers.getLatency_Pipeline(name);
    }

    public double[] getVisionPose(String name) {
        Pose2d pose = LimelightHelpers.getBotPose2d(name);
        double[] list = {pose.getX(), pose.getY(), pose.getRotation().getDegrees()};

        return list;
    }

    public Optional<VisionMeasurement> getVisionMeasurement(String name) {

        if (!LimelightHelpers.getTV(name)) {
            return Optional.empty();
        }

        Pose2d visionPose = LimelightHelpers.getBotPose2d_wpiBlue(name);
        double latencyMs = LimelightHelpers.getLatency_Capture(name) + LimelightHelpers.getLatency_Pipeline(name);
        double timestampSeconds = Timer.getFPGATimestamp() - (latencyMs / 1000.0);
        int tagCount = LimelightHelpers.getTargetCount(name);

        Matrix<N3, N1> stdDevs = 
            tagCount >= 2
                ?VisionConstants.kMultiTagStdDevs
                :VisionConstants.kSingleTagStdDevs;

        VisionMeasurement measurement = new VisionMeasurement(visionPose, timestampSeconds, stdDevs);

        return Optional.of(measurement);

    }

    @Override
    public void periodic() {
        String name = VisionConstants.kFrontLimelightName;

        SmartDashboard.putBoolean("Vision/VisionValid?", hasValidPose(name));

        SmartDashboard.putBoolean("Vision/TV", LimelightHelpers.getTV(name));

        SmartDashboard.putNumber("Vision/TagCount", getTagCount(name));

        SmartDashboard.putNumber("Vision/LatencySec", getLatencySeconds(name) / 1000.0);

        getVisionMeasurement(name).ifPresent(m -> {
            Pose2d p = m.getPose();
            SmartDashboard.putNumber("Vision/PoseX", p.getX());
            SmartDashboard.putNumber("Vision/PoseY", p.getY());
            SmartDashboard.putNumber("Vision/PoseDeg", p.getRotation().getDegrees());
        });
    }
}