/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.launcher;

import static org.robolancers321.util.MathUtils.epsilonEquals;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.NavigableSet;
import java.util.TreeSet;

public class AimTable {
  public static class AimCharacteristic {
    public final double angle;
    public final double rpm;

    public AimCharacteristic(double angle, double rpm) {
      this.angle = angle;
      this.rpm = rpm;
    }
  }

  private AimCharacteristic lastAimCharacteristic;

  private final double lastDistance = 0;

  private double kInterpolationCacheThreshold = 0.0;

  // TODO: tune
  public static AimCharacteristic kRetractedAimCharacteristic = new AimCharacteristic(0.0, 0.0);
  public static AimCharacteristic kMatingAimCharacteristic = new AimCharacteristic(0.0, 0.0);
  public static AimCharacteristic kAmpAimCharacteristic = new AimCharacteristic(0.0, 0.0);

  // TODO: tune
  private static final LinkedHashMap<Double, AimCharacteristic> table =
      new LinkedHashMap<>() {
        {
          put(0.0, new AimCharacteristic(0.0, 0.0));
          put(1.0, new AimCharacteristic(0.0, 0.0));
        }
      };

  private static double interpolate(
      double lowKey, double lowValue, double highKey, double highValue, double x) {
    double percent = (x - lowKey) / (highKey - lowKey);

    return lowKey + percent * (highValue - lowValue);
  }

  public static AimCharacteristic getSpeakerAimCharacteristic(double distance) {
    List<Double> keys = table.keySet().stream().sorted().toList();
    double lowerBound = keys.get(0);
    double upperBound = keys.get(keys.size() - 1);

    if ((distance < lowerBound)) {
      AimTable.AimCharacteristic lowerValue = table.get(lowerBound);

      return new AimTable.AimCharacteristic(lowerValue.angle, lowerValue.rpm);
    } else if (distance > upperBound) {
      AimTable.AimCharacteristic upperValue = table.get(upperBound);

      return new AimTable.AimCharacteristic(upperValue.angle, upperValue.rpm);
    }

    NavigableSet<Double> values = new TreeSet<>(keys);
    double lowKey = values.floor(distance);
    double highKey = values.ceiling(distance);

    return new AimCharacteristic(
        interpolate(lowKey, table.get(lowKey).angle, highKey, table.get(highKey).angle, distance),
        interpolate(lowKey, table.get(lowKey).rpm, highKey, table.get(highKey).rpm, distance));
  }

  public AimCharacteristic getLastAimCharacteristic(double distance) {
    if (!epsilonEquals(lastDistance, distance, kInterpolationCacheThreshold)) {
      lastAimCharacteristic = getSpeakerAimCharacteristic(lastDistance);
      kInterpolationCacheThreshold = lastDistance;
    }

    return lastAimCharacteristic;
  }
}
