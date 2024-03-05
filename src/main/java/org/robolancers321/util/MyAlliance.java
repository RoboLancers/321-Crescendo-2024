/* (C) Robolancers 2024 */
package org.robolancers321.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

public class MyAlliance {
  public static boolean isRed() {
    Optional<Alliance> myAlliance = DriverStation.getAlliance();

    return myAlliance.isPresent() && myAlliance.get() == DriverStation.Alliance.Red;
  }
}
