/* (C) Robolancers 2024 */
package org.robolancers321.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.DoubleSupplier;

// Stolen from Matt's swerve repo
public class TunableConstant {
  // whether constants should globally stream or not, should be false unless tuning
  private static final boolean TUNABLE = true;

  private final DoubleSupplier getter;

  public TunableConstant(String key, double defaultValue) {
    // if and only if no constant is there already, set it to the default value
    SmartDashboard.putNumber(key, SmartDashboard.getNumber(key, defaultValue));

    if (TunableConstant.TUNABLE) {
      this.getter = () -> SmartDashboard.getNumber(key, defaultValue);
    } else {
      this.getter = () -> defaultValue;
    }
  }

  public double get() {
    return this.getter.getAsDouble();
  }
}
