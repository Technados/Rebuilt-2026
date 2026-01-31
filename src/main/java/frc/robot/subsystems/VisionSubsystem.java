package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.FrontLimelightConstants;
import frc.robot.Constants.BackLimelightConstants;
import frc.robot.Constants.DriveConstants;

public class VisionSubsystem extends SubsystemBase {

    NetworkTable table;
    public static PIDController rotationPID = createPIDController();

    private final double limelightHeight;
    private final double limelightMountAngle;

    private final PIDController limelightTurnPID;
    private final PIDController limelightStrafePID;
    private final PIDController limelightDistancePID;

    private final String limelightName;

    public VisionSubsystem(string limelightName) {
        this.limelightName = limelightName;

        table = NetworkTableInstance.getDefault().getTable(limelightName);

        /* If the limelights have different functions, it might make more sense
        to make a limelight abstract class and have two seperate classes for
        front and back, but this should work if not */ 
        if (this.limelightName.equals("frontLimelight")) { // Change names to the names set in Network Table
            limelightHeight = kFrontLimelightHeight;
            limelightMountAngle = kFrontLimelightMountAngle;
        }

        if (this.limelightName.equals("backLimelight")) { // Change names to the names set in Network Table
            limelightHeight = kBackLimelightHeight;
            limelightMountAngle = kBackLimelightMountAngle;
        }

        PIDController limelightTurnPID = new PIDController(
            Constants.LimelightPID.kP_turn, 
            Constants.LimelightPID.kI_turn, 
            Constants.LimelightPID.kD_turn
        );

        PIDController limelightStrafePID = new PIDController(
            Constants.LimelightPID.kP_strafe, 
            Constants.LimelightPID.kI_strafe, 
            Constants.LimelightPID.kD_strafe
        );

        PIDController limelightDistancePID = new PIDController(
            Constants.LimelightPID.kP_distance, 
            Constants.LimelightPID.kI_distance, 
            Constants.LimelightPID.kD_distance
        );
    }

    public double getR() {
        return limelight_aim_proportional();
        // return rotationPID.calculate(table.getEntry("tx").getDouble(0));
    }

    public double getXSpeed() {
        return limelight_range_proportional();
    }

    public double getDistance(double targetHeight) {
        Rotation2d angleToGoal = Rotation2d.fromDegrees(limelightMountAngle)
        .plus(Rotation2d.fromDegrees(LimelightHelpers.getTX(limelightName)));
        double distance = (targetHeight - limelightMountAngle) / angleToGoal.getTan();
        return distance;
    }

    public double getDistanceToApriltag() {
        Pose3d targetPoseRobotSpace = LimelightHelpers.getTargetPose3d_RobotSpace();

        double x = targetPoseRobotSpace.getX();
        double z = targetPoseRobotSpace.getZ();

        return Math.hypot(x, z);
    }

    public double getRotation() {
        double cameraLensHorizontalOffset = LimelightHelpers.getTX(limelightName) / getDistance();
        double realHorizontalOffset = Math.atan(cameraLensHorizontalOffset / getDistance());
        double rotationError = Math.atan(realHorizontalOffset / getDistance());
        return rotationError;
    }

    public double limelight_aim_proportional() {
        /* kP (constant of proportionality)
        this is a hand-tuned number that determines the aggressiveness of our
        proportional control loop
        if it is too high, the robot will oscillate around.
        if it is too low, the robot will never reach its target
        if the robot never turns in the correct direction, kP should be inverted. */ 
        double kP = .005;

        /* tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the
        rightmost edge of
        your limelight 3 feed, tx should return roughly 31 degrees. */
        double targetingAngularVelocity = table.getEntry("tx").getDouble(0) * kP;

        // convert to radians per second for our drive method
        targetingAngularVelocity *= Constants.DriveConstants.kMaxAngularSpeed / 4;

        // invert since tx is positive when the target is to the right of the crosshair
        targetingAngularVelocity *= -1.0;

        return targetingAngularVelocity;
    }

    public double limelight_range_proportional() {
        double kP = 0.0095;

        double targetingForwardSpeed = table.getEntry("ty").getDouble(0) * kP;
        targetingForwardSpeed *= Constants.DriveConstants.kMaxSpeedMetersPerSecond;
        return targetingForwardSpeed;

    }
}