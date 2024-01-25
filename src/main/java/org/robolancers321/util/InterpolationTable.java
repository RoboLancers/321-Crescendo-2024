/* (C) Robolancers 2024 */
package org.robolancers321.util;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.NavigableSet;
import java.util.TreeSet;

public class InterpolationTable {
  public static final class AimCharacteristic {
    private final double leftSpeed;
    private final double rightSpeed;
    private final double pitchAngle;

    public AimCharacteristic(double leftSpeed, double rightSpeed, double pitchAngle) {
      this.leftSpeed = leftSpeed;
      this.rightSpeed = rightSpeed;
      this.pitchAngle = pitchAngle;
    }

    public double getLeftSpeed() {
      return leftSpeed;
    }

    public double getRightSpeed() {
      return rightSpeed;
    }

    public double getPitchAngle() {
      return pitchAngle;
    }
  }

  public static LinkedHashMap<Double, AimCharacteristic> table =
      new LinkedHashMap<>() {
        {
          put(1.0, new AimCharacteristic(1000, 1000, 10));
          put(2.0, new AimCharacteristic(2000, 2000, 20));
        }
      };

  public static AimCharacteristic interpolate(double independent) {
    List<Double> keys = table.keySet().stream().toList();
    double lowerBound = keys.get(0);
    double upperBound = keys.get(keys.size() - 1);

    if ((independent < lowerBound)) independent = lowerBound;
    else if (independent > upperBound) independent = upperBound;

    NavigableSet<Double> values = new TreeSet<>(keys);

    double lowerKey = values.floor(independent);
    double upperKey = lowerKey + 1;

    AimCharacteristic lowerValue = table.get(lowerKey);
    AimCharacteristic upperValue = table.get(upperKey);

    double leftSpeed =
        calculateCharacteristic(
            lowerKey, upperKey, lowerValue.getLeftSpeed(), upperValue.getLeftSpeed(), independent);

    double rightSpeed =
        calculateCharacteristic(
            lowerKey,
            upperKey,
            lowerValue.getRightSpeed(),
            upperValue.getRightSpeed(),
            independent);

    double pitchAngle =
        calculateCharacteristic(
            lowerKey,
            upperKey,
            lowerValue.getPitchAngle(),
            upperValue.getPitchAngle(),
            independent);

    return new AimCharacteristic(leftSpeed, rightSpeed, pitchAngle);
  }

  private static double calculateCharacteristic(
      double lowerKey, double upperKey, double lower, double upper, double independent) {
    return ((lower * (upperKey - independent) + upper * (independent - lowerKey)) / upperKey
        - lowerKey);
  }
}
