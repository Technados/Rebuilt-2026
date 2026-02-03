package frc.robot.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionMeasurement {

    private Pose2d pose;
    private double timestampSeconds;
    private Matrix<N3, N1> stdDevs;
    
    public VisionMeasurement(Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
        this.pose = pose;
        this.timestampSeconds = timestampSeconds;
        this.stdDevs = stdDevs;

        stdDevs.set(0, 0, this.pose.getX());
        stdDevs.set(1, 0, this.pose.getY());
        stdDevs.set(2, 0, this.pose.getRotation().getDegrees());
    }

    public Pose2d getPose() {
        return pose;
    }

    public double getTimestampSeconds() {
        return timestampSeconds;
    }

    public Matrix<N3, N1> getStdDevs() {
        return stdDevs;
    }

}
