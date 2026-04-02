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
import frc.robot.FieldConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.vision.VisionMeasurement;

public class VisionSubsystem extends SubsystemBase {

    public VisionSubsystem() {}

    /*----------Getters----------*/

    /**
     * Returns if the limelight's has a valid pose.
     * @param name The name of the limelight.
     * @return Returns true if the limelight's pose estimate is valid.
     */
    public boolean hasValidPose(String name) { // Returns true if limelight has a valid pose estimate
        return LimelightHelpers.validPoseEstimate(LimelightHelpers.getBotPoseEstimate_wpiBlue(name));
    }

    /**
     * Get the limelight's tag count.
     * @param name The name of the limelight.
     * @return How many targets the limelight can see.
     */
    public int getTagCount(String name) { // Gets the limelight's tag count
        return LimelightHelpers.getTargetCount(name);
    }

    /**
     * Gets the limelight's latency.
     * @param name The name of the limelight.
     * @return The limelight's latency (capture latency + pipeline latency)
     */
    public double getLatencySeconds(String name) {
        return LimelightHelpers.getLatency_Capture(name) + LimelightHelpers.getLatency_Pipeline(name);
    }

    public String[] getLimelightNames(boolean preciseMode) {
        String[] llnames =
            preciseMode
              ? new String[] {getLimelightWithMostTags()}
              : new String[] {
                  Constants.VisionConstants.kFrontLimelightName,
                  Constants.VisionConstants.kBackLimelightName
                };

        return llnames;
    }

    public String getLimelightWithMostTags() {
        String llnames = 
            getTagCount(VisionConstants.kFrontLimelightName) == getTagCount(VisionConstants.kBackLimelightName)
            ? VisionConstants.kFrontLimelightName
            : getTagCount(VisionConstants.kFrontLimelightName) > getTagCount(VisionConstants.kBackLimelightName)
                ? VisionConstants.kFrontLimelightName
                : VisionConstants.kBackLimelightName;

        return llnames;
    }

    public boolean rejectVisionMeasurement(LimelightHelpers.PoseEstimate mt2, boolean preciseMode) {
        boolean tagCountValid =
            preciseMode
                ? mt2.tagCount < 2
                : mt2.tagCount <= 0;

        boolean rejectPose =
            // Check if measurement is valid
            mt2 == null ||
            tagCountValid ||

            // Check if measurement is outside the field
            mt2.pose.getX() < (0 + ModuleConstants.kBotCenterOffsetMeters) - VisionConstants.kWithinFieldToleranceMeters ||
            mt2.pose.getX() > (FieldConstants.FIELD_LENGTH_M - ModuleConstants.kBotCenterOffsetMeters) + VisionConstants.kWithinFieldToleranceMeters ||
            mt2.pose.getY() < (0 + ModuleConstants.kBotCenterOffsetMeters) - VisionConstants.kWithinFieldToleranceMeters ||
            mt2.pose.getY() > (FieldConstants.FIELD_WIDTH_M - ModuleConstants.kBotCenterOffsetMeters) + VisionConstants.kWithinFieldToleranceMeters ||

            // Check if measurement is inside the hub
            (mt2.pose.getX() > FieldConstants.getHubXMin() && mt2.pose.getY() > FieldConstants.getHubYMin() &&
            mt2.pose.getX() < FieldConstants.getHubXMax() && mt2.pose.getY() < FieldConstants.getHubYMax());

        return rejectPose;
    }

    /**
     * Gets vision measurements from a limelight.
     * @param name The name of the limelight.
     * @return A {@link VisionMeasurement} with the calculated pose, timestamp, and std devs.
     */
    public Optional<VisionMeasurement> getVisionMeasurement(String name) {

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

    public Optional<VisionMeasurement> getVisionMeasurementMT2(
    String name,
    double robotYawDeg,
    double robotYawRateDegPerSec,
    double robotPitchDeg,
    double robotRollDeg,
    boolean recoveryMode) {

        // Set current robot orientation for MegaTag2
        LimelightHelpers.SetRobotOrientation(
            name, 
            robotYawDeg, 
            robotYawRateDegPerSec, 
            robotPitchDeg, 
            0, 
            robotRollDeg, 
            0
        );

        LimelightHelpers.PoseEstimate mt2 =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

        if (rejectVisionMeasurement(mt2, recoveryMode)) {
            return Optional.empty();
        }

        Matrix<N3, N1> stdDevs;

        if (recoveryMode) {
            stdDevs = VisionConstants.kRecoveryMultiTagStdDevs;
        } else {
            stdDevs =
                mt2.tagCount >= 2
                    ? VisionConstants.kMultiTagStdDevs
                    : VisionConstants.kSingleTagStdDevs;
        }

        VisionMeasurement measurement =
            new VisionMeasurement(mt2.pose, mt2.timestampSeconds, stdDevs);

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