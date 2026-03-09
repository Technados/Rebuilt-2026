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
        new ShotMap.ShotPoint(1.4321, 3500, 0.20),
        new ShotMap.ShotPoint(2.918, 4000, 0.38),
        new ShotMap.ShotPoint(5.6358, 4250, 0.55)
    );

    public static ShotMap createAllianceZoneShotMap() {
        return new ShotMap(ALLIANCE_ZONE_POINTS);
    }

}
