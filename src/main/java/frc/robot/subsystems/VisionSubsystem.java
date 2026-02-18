package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.vision.VisionMeasurement;

public class VisionSubsystem extends SubsystemBase {

    public VisionSubsystem() {}

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

    // Gets vision measurements from the limelight 
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

    // Gets vision measurements using MegaTag2
    public Optional<VisionMeasurement> getVisionMeasurementMT2(String name, double robotYawDeg, double robotYawRateDegPerSec) {

        LimelightHelpers.SetRobotOrientation(name, robotYawRateDegPerSec, robotYawRateDegPerSec, 0, 0, 0, 0);

        LimelightHelpers.PoseEstimate mt2 = 
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

        if (mt2 == null || mt2.tagCount <= 0) {
            return Optional.empty();
        }

        Matrix<N3, N1> stdDevs = 
            mt2.tagCount >= 2
                ?VisionConstants.kMultiTagStdDevs
                :VisionConstants.kSingleTagStdDevs;

        VisionMeasurement measurement = new VisionMeasurement(mt2.pose, mt2.timestampSeconds, stdDevs);

        return Optional.of(measurement);

    }

    @Override
    public void periodic() {
        String name = VisionConstants.kFrontLimelightName;

        SmartDashboard.putBoolean("Vision/TV", LimelightHelpers.getTV(name));
        SmartDashboard.putNumber("Vision/TagCount", getTagCount(name));

        LimelightHelpers.PoseEstimate mt2 = 
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name); 
    
        boolean hasTags = (mt2 != null && mt2.tagCount > 0); 
        SmartDashboard.putBoolean("Vision/MT2 HasTags", hasTags); 

        if (hasTags) { 
            Pose2d p = mt2.pose; 
            SmartDashboard.putNumber("Vision/MT2 PoseX", p.getX()); 
            SmartDashboard.putNumber("Vision/MT2 PoseY", p.getY()); 
            SmartDashboard.putNumber("Vision/MT2 PoseDeg", p.getRotation().getDegrees()); 
            SmartDashboard.putNumber("Vision/MT2 Timestamp", mt2.timestampSeconds);
        }
    }
}