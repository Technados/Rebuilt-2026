package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

/**
* Field geometry constants used for pose-based calculations.
*
* WPILib field coordinates (2024+ convention):
* - Origin is the BLUE alliance corner
* - +X points toward RED alliance wall
* - +Y points left when standing at the BLUE wall facing the field
*
* This field is rotationally symmetric (180° rotation), not mirrored, so we
* compute the RED hub by rotating the BLUE hub around the field center.
*/

public final class FieldConstants {

    private FieldConstants() {}

    // Welded field size (meters)
    public static final double FIELD_LENGTH_M = 16.541; // 651.22 in
    public static final double FIELD_WIDTH_M  = 8.069;  // ~317.69 in

    // Hub centers (meters) derived from welded hub AprilTag ring centroid
    public static final Translation2d BLUE_HUB = new Translation2d(4.5366305, 4.0345995);

    public static final double BLUE_HUB_X_MIN = 4.0287;
    public static final double BLUE_HUB_X_MAX = 5.2225;
    public static final double BLUE_HUB_Y_MIN = 3.4377;
    public static final double BLUE_HUB_Y_MAX = 4.6316;
    
    // Rotated 180° about field center
    public static final Translation2d RED_HUB = new Translation2d(
        FIELD_LENGTH_M - BLUE_HUB.getX(),
        FIELD_WIDTH_M  - BLUE_HUB.getY()
        );

    public static final double RED_HUB_X_MIN = 11.3185;
    public static final double RED_HUB_X_MAX = 12.5123;
    public static final double RED_HUB_Y_MIN = 3.4377;
    public static final double RED_HUB_Y_MAX = 4.6316;
        
    public static final Translation2d BLUE_LEFT_FIELD_TARGET = new Translation2d(1.989, 1.849);
    public static final Translation2d BLUE_RIGHT_FIELD_TARGET = new Translation2d(1.989, 6.220);

    public static final Translation2d RED_LEFT_FIELD_TARGET = new Translation2d(
        FIELD_LENGTH_M - BLUE_LEFT_FIELD_TARGET.getX(),
        FIELD_WIDTH_M  - BLUE_LEFT_FIELD_TARGET.getY()
    );
    public static final Translation2d RED_RIGHT_FIELD_TARGET = new Translation2d(
        FIELD_LENGTH_M - BLUE_RIGHT_FIELD_TARGET.getX(),
        FIELD_WIDTH_M  - BLUE_RIGHT_FIELD_TARGET.getY()
    );

    public static final Pose2d RED_TEST_POSE = new Pose2d(
        12.928, 4, Rotation2d.fromDegrees(0)
    );
    public static final Pose2d BLUE_TEST_POSE = new Pose2d(
        (BLUE_HUB.getX() - .9236305), 4, Rotation2d.fromDegrees(180)
    );

    /** Returns the hub your alliance should score into, in WPILib field coordinates. */
    public static Translation2d getAllianceHub() {
        var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        return (alliance == DriverStation.Alliance.Blue) ? BLUE_HUB : RED_HUB;
    }

    public static Translation2d getLeftFieldTarget() {
        var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        return (alliance == DriverStation.Alliance.Blue) ? BLUE_LEFT_FIELD_TARGET : RED_LEFT_FIELD_TARGET;
    }

    public static Translation2d getRightFieldTarget() {
        var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        return (alliance == DriverStation.Alliance.Blue) ? BLUE_RIGHT_FIELD_TARGET : RED_RIGHT_FIELD_TARGET;
    }

    public static Pose2d getRedTestPose() {
        return RED_TEST_POSE;
    }

    public static Pose2d getBlueTestPose() {
        return BLUE_TEST_POSE;
    }

    public static double getHubXMin() {
        var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        return (alliance == DriverStation.Alliance.Blue) ? BLUE_HUB_X_MIN : RED_HUB_X_MIN;
    }

    public static double getHubXMax() {
        var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        return (alliance == DriverStation.Alliance.Blue) ? BLUE_HUB_X_MAX : RED_HUB_X_MAX;
    }

    public static double getHubYMin() {
        var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        return (alliance == DriverStation.Alliance.Blue) ? BLUE_HUB_Y_MIN : RED_HUB_Y_MIN;
    }

    public static double getHubYMax() {
        var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        return (alliance == DriverStation.Alliance.Blue) ? BLUE_HUB_Y_MAX : RED_HUB_Y_MAX;
    }

}
