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

  private static final double kInterpolationCacheThreshold = 0.0;

  private static AimCharacteristic calculateSpeakerAimCharacteristic(double distance) {
    List<Double> keys = table.keySet().stream().sorted().toList();
    double lowerBound = keys.get(0);
    double upperBound = keys.get(keys.size() - 1);

    if (distance < lowerBound) {
      AimTable.AimCharacteristic lowerValue = table.get(lowerBound);

      return new AimTable.AimCharacteristic(lowerValue.angle, lowerValue.rpm);
    }

    if (distance > upperBound) {
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

  private double lastDistance;
  private AimCharacteristic lastAimCharacteristic;

  public AimTable() {
    this.lastDistance = 0.0;
    this.lastAimCharacteristic = new AimCharacteristic(0.0, 0.0);
  }

  public void updateSpeakerAimCharacteristic(double distance) {
    if (!epsilonEquals(this.lastDistance, distance, kInterpolationCacheThreshold)) {
      this.lastDistance = distance;
      this.lastAimCharacteristic = calculateSpeakerAimCharacteristic(distance);
    }
  }

  public AimCharacteristic getSpeakerAimCharacteristic() {
    return this.lastAimCharacteristic;
  }
}
