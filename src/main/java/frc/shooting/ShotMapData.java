package frc.shooting;

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
        new ShotMap.ShotPoint(2.0, 2600, 0.22),
        new ShotMap.ShotPoint(3.0, 3100, 0.30),
        new ShotMap.ShotPoint(4.0, 3600, 0.41),
        new ShotMap.ShotPoint(5.0, 4200, 0.53),
        new ShotMap.ShotPoint(6.0, 4800, 0.64)
    );

    public static ShotMap createAllianceZoneShotMap() {
        return new ShotMap(ALLIANCE_ZONE_POINTS);
    }

}
