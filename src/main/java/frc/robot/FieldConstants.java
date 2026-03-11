package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
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

    // Rotated 180° about field center
    public static final Translation2d RED_HUB = new Translation2d(
        FIELD_LENGTH_M - BLUE_HUB.getX(),
        FIELD_WIDTH_M  - BLUE_HUB.getY()
    );

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

    /** Returns the hub your alliance should score into, in WPILib field coordinates. */
    public static Translation2d getAllianceHub() {
        var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        return (alliance == DriverStation.Alliance.Blue) ? BLUE_HUB : RED_HUB;
    }

    public static Translation2d getAllianceFieldTarget(Pose2d pose) {
        var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);

        if (alliance == DriverStation.Alliance.Blue) {
            if (FIELD_WIDTH_M / 2 > pose.getY()) {
                return BLUE_LEFT_FIELD_TARGET;
            } else {
                return BLUE_RIGHT_FIELD_TARGET;
            }
        } else {
            if (FIELD_WIDTH_M / 2 < pose.getY()) {
                return BLUE_LEFT_FIELD_TARGET;
            } else {
                return BLUE_RIGHT_FIELD_TARGET;
            }
        }
    }

}
