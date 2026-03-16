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
        new ShotMap.ShotPoint(1, 2800, 0.2),
        new ShotMap.ShotPoint(1.5, 3050, 0.3),
        new ShotMap.ShotPoint(2.2, 3300, 0.37),
        new ShotMap.ShotPoint(2.5, 3550, 0.42)
    );

    public static ShotMap createAllianceZoneShotMap() {
        return new ShotMap(ALLIANCE_ZONE_POINTS);
    }

}
