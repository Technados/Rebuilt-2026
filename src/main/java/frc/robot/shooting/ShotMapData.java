package frc.robot.shooting;

import java.util.List;

/**
* This file is intentionally "boring data".
* After shot testing, you only edit these points (distance, rpm, hoodPos).
*/

public final class ShotMapData {

    private ShotMapData() {}
    
    // Sample placeholder values to prove the system works end-to-end.
    // Replace with real measured data.
    public static final List<ShotMap.ShotPoint> ALLIANCE_ZONE_POINTS = List.of(
        new ShotMap.ShotPoint(1.17, 2300, 0.2),
        new ShotMap.ShotPoint(1.5, 2500, 0.3),
        new ShotMap.ShotPoint(2.05, 2750, 0.35),
        new ShotMap.ShotPoint(2.5, 2900, 0.36),
        new ShotMap.ShotPoint(3.5, 3050, 0.43),
        new ShotMap.ShotPoint(4, 3300, 0.48)
    );

    public static ShotMap createAllianceZoneShotMap() {
        return new ShotMap(ALLIANCE_ZONE_POINTS);
    }

}
