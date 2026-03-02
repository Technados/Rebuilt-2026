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
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.vision.VisionMeasurement;

public class VisionSubsystem extends SubsystemBase {

    public VisionSubsystem() {}

    /*----------Getters----------*/

    public boolean hasValidPose(String name) { // Returns true if limelight has a valid pose estimate
        return LimelightHelpers.validPoseEstimate(LimelightHelpers.getBotPoseEstimate_wpiBlue(name));
    }

    public int getTagCount(String name) { // Gets the limelight's tag count
        return LimelightHelpers.getTargetCount(name);
    }

    public double getLatencySeconds(String name) { // Gets the limelight's latency
        return LimelightHelpers.getLatency_Capture(name) + LimelightHelpers.getLatency_Pipeline(name);
    }

    public double[] getVisionPose(String name) {  // Gets the limelight's pose as a list (x, y, rot)
        Pose2d pose = LimelightHelpers.getBotPose2d(name);
        double[] list = {pose.getX(), pose.getY(), pose.getRotation().getDegrees()};

        return list;
    }

    public Optional<VisionMeasurement> getVisionMeasurement(String name) { // Gets vision measurements from the limelight 

        // Check if there is a valid target
        if (!LimelightHelpers.getTV(name)) {
            return Optional.empty();
        }

        // Get pose, latency, timestamp, and tag count
        Pose2d visionPose = LimelightHelpers.getBotPose2d_wpiBlue(name);
        double latencyMs = LimelightHelpers.getLatency_Capture(name) + LimelightHelpers.getLatency_Pipeline(name);
        double timestampSeconds = Timer.getFPGATimestamp() - (latencyMs / 1000.0);
        int tagCount = LimelightHelpers.getTargetCount(name);

        // Choose std devs (confidence)
        Matrix<N3, N1> stdDevs = 
            tagCount >= 2
                ?VisionConstants.kMultiTagStdDevs
                :VisionConstants.kSingleTagStdDevs;

        // Return new VisionMeasurement
        VisionMeasurement measurement = new VisionMeasurement(visionPose, timestampSeconds, stdDevs);

        return Optional.of(measurement);

    }

    public Optional<VisionMeasurement> getVisionMeasurementMT2(String name, double robotYawDeg, double robotYawRateDegPerSec) { // Gets vision measurements using MegaTag2

        // Set current robot orientation
        LimelightHelpers.SetRobotOrientation(name, robotYawDeg, robotYawRateDegPerSec, 0, 0, 0, 0);

        // Get mt2 pose estimate
        LimelightHelpers.PoseEstimate mt2 = 
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

        // Check if mt2 is valid
        if (mt2 == null || mt2.tagCount <= 0) {
            return Optional.empty();
        }

        // Choose std devs (confidence)
        Matrix<N3, N1> stdDevs = 
            mt2.tagCount >= 2
                ?VisionConstants.kMultiTagStdDevs
                :VisionConstants.kSingleTagStdDevs;

        // Return new mt2 vision measurement
        VisionMeasurement measurement = new VisionMeasurement(mt2.pose, mt2.timestampSeconds, stdDevs);

        return Optional.of(measurement);

    }

    /*----------Periodic----------*/

    @Override
    public void periodic() {
        String[] llnames = {Constants.VisionConstants.kFrontLimelightName, Constants.VisionConstants.kBackLimelightName};

        for (String name: llnames) {
            SmartDashboard.putBoolean("Vision/" + name + "/TV", LimelightHelpers.getTV(name));
            SmartDashboard.putNumber("Vision/" + name + "/TagCount", getTagCount(name));

            LimelightHelpers.PoseEstimate mt2 = 
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name); 
        
            boolean hasTags = (mt2 != null && mt2.tagCount > 0); 
            SmartDashboard.putBoolean("Vision/" + name + "/MT2 HasTags", hasTags); 

            if (hasTags) { 
                Pose2d p = mt2.pose; 
                SmartDashboard.putNumber("Vision/" + name + "/MT2 PoseX", p.getX()); 
                SmartDashboard.putNumber("Vision/" + name + "/MT2 PoseY", p.getY()); 
                SmartDashboard.putNumber("Vision/" + name + "/MT2 PoseDeg", p.getRotation().getDegrees()); 
                SmartDashboard.putNumber("Vision/" + name + "/MT2 Timestamp", mt2.timestampSeconds);
            }
        }
    }
}