package frc.robot.shooting;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/**
* ShotMap interpolates between empirical data points:
* distance (meters) -> (shooter RPM, hood position)
*
* Why interpolation?
* - avoids "steppy" behavior when distance changes slightly
* - gives smoother and more repeatable shots
* - makes your table smaller (you don’t need a row every 0.05m)
*/

public class ShotMap {

    public record ShotPoint(double distanceM, double rpm, double hoodPos) {}

    private final List<ShotPoint> points;

    public ShotMap(List<ShotPoint> points) {
        this.points = new ArrayList<>(points);
        this.points.sort(Comparator.comparingDouble(ShotPoint::distanceM));
    }

    public ShotParameters get(double distanceM) {
        if (points.isEmpty()) return new ShotParameters(0.0, 0.0);

        // Clamp to table endpoints
        if (distanceM <= points.get(0).distanceM()) {
            var p = points.get(0);
            return new ShotParameters(p.rpm(), p.hoodPos());
        }
        var last = points.get(points.size() - 1);
        if (distanceM >= last.distanceM()) {
            return new ShotParameters(last.rpm(), last.hoodPos());
        }

        // Find adjacent points
        ShotPoint a = points.get(0), b = points.get(1);
        for (int i = 0; i < points.size() - 1; i++) {
            var p0 = points.get(i);
            var p1 = points.get(i + 1);
            if (distanceM >= p0.distanceM() && distanceM <= p1.distanceM()) {
            a = p0;
            b = p1;
            break;
            }
        }

        // Linear interpolation
        double t = (distanceM - a.distanceM()) / (b.distanceM() - a.distanceM());
        double rpm = a.rpm() + (b.rpm() - a.rpm()) * t;
        double hood = a.hoodPos() + (b.hoodPos() - a.hoodPos()) * t;

        return new ShotParameters(rpm, hood);
    }

}
