package frc.robot.distance;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;

public class DistanceMeasurement {

    Supplier<Translation2d> hubSupplier;
    Supplier<Translation2d> leftFieldTargetSupplier;
    Supplier<Translation2d> rightFieldTargetSupplier;

    DoubleSupplier hubDistanceSupplier;
    DoubleSupplier leftFieldDistanceSupplier;
    DoubleSupplier rightFieldDistanceSupplier;

    public DistanceMeasurement(
        Supplier<Translation2d> hubSupplier, 
        Supplier<Translation2d> leftFieldTargetSupplier, 
        Supplier<Translation2d> rightFieldTargetSupplier,

        DoubleSupplier hubDistanceSupplier, 
        DoubleSupplier leftFieldDistanceSupplier, 
        DoubleSupplier rightFieldDistanceSupplier
    ) {

        this.hubSupplier = hubSupplier;
        this.leftFieldTargetSupplier = leftFieldTargetSupplier;
        this.rightFieldTargetSupplier = rightFieldTargetSupplier;

        this.hubDistanceSupplier = hubDistanceSupplier;
        this.leftFieldDistanceSupplier = leftFieldDistanceSupplier;
        this.rightFieldDistanceSupplier = rightFieldDistanceSupplier;
    }

    public Supplier<Translation2d> getHubSupplier() {
        return hubSupplier;
    }

    public Supplier<Translation2d> getLeftFieldTargetSupplier() {
        return leftFieldTargetSupplier;
    }

    public Supplier<Translation2d> getRightFieldTargetSupplier() {
        return rightFieldTargetSupplier;
    }

    public DoubleSupplier getHubDistanceSupplier() {
        return hubDistanceSupplier;
    }

    public DoubleSupplier getLeftFieldDistanceSupplier() {
        return leftFieldDistanceSupplier;
    }

    public DoubleSupplier getRightFieldDistanceSupplier() {
        return rightFieldDistanceSupplier;
    }
}
