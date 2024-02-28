package org.robolancers321.util;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class MyAlliance {
    public static boolean isRed(){
        Optional<Alliance> myAlliance = DriverStation.getAlliance();

        return myAlliance.isPresent() && myAlliance.get() == DriverStation.Alliance.Red;
    }
}
