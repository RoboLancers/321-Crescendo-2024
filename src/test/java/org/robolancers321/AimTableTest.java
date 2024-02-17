/* (C) Robolancers 2024 */
package org.robolancers321;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.robolancers321.util.MathUtils.epsilonEquals;

import java.util.*;
import org.junit.jupiter.api.Test;
import org.robolancers321.subsystems.launcher.AimTable;

public class AimTableTest {
  private static final LinkedHashMap<Double, AimTable.AimCharacteristic> table =
      new LinkedHashMap<>() {
        {
          put(0.0, new AimTable.AimCharacteristic(0, 0.0));
          put(1.0, new AimTable.AimCharacteristic(10, 1020));
          put(2.0, new AimTable.AimCharacteristic(20, 1500));
          put(4.0, new AimTable.AimCharacteristic(40, 2500));
          put(5.0, new AimTable.AimCharacteristic(54, 3300));
          put(6.0, new AimTable.AimCharacteristic(61, 4227));
          put(8.0, new AimTable.AimCharacteristic(83, 5599));
        }
      };

  private class DummyAimTable {
    private AimTable.AimCharacteristic lastAimCharacteristic;

    private final double lastDistance = 0;

    private double kInterpolationCacheThreshold = 0.0;

    private static double interpolate(
        double lowKey, double lowValue, double highKey, double highValue, double x) {
      double percent = (x - lowKey) / (highKey - lowKey);

      return lowValue + percent * (highValue - lowValue);
    }

    public static AimTable.AimCharacteristic getSpeakerAimCharacteristic(double distance) {
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

      System.out.println("Distance " + distance);

      NavigableSet<Double> values = new TreeSet<>(keys);
      double lowKey = values.floor(distance);
      double highKey = values.ceiling(distance);

      return new AimTable.AimCharacteristic(
          interpolate(lowKey, table.get(lowKey).angle, highKey, table.get(highKey).angle, distance),
          interpolate(lowKey, table.get(lowKey).rpm, highKey, table.get(highKey).rpm, distance));
    }

    public AimTable.AimCharacteristic getLastAimCharacteristic(double distance) {
      if (!epsilonEquals(lastDistance, distance, kInterpolationCacheThreshold)) {
        lastAimCharacteristic = getSpeakerAimCharacteristic(lastDistance);
        kInterpolationCacheThreshold = lastDistance;
      }

      return lastAimCharacteristic;
    }
  }

  private List<Double> getKeys() {
    return table.keySet().stream().sorted().toList();
  }

  private static double interpolate(
      double lowKey, double lowValue, double highKey, double highValue, double x) {
    double percent = (x - lowKey) / (highKey - lowKey);

    return lowValue + percent * (highValue - lowValue);
  }

  @Test
  void testKeys() {
    var keys = getKeys();

    var expectedKeys =
        new Double[] {
          0.0, 1.0, 2.0, 4.0, 5.0, 6.0, 8.0,
        };

    assertArrayEquals(keys.toArray(new Double[0]), expectedKeys);
  }

  @Test
  void testUpperBoundKey() {
    var keys = getKeys();
    double upperBound = keys.get(keys.size() - 1);

    assertEquals(14.0, upperBound);
  }

  @Test
  void testLowerBoundKey() {
    var keys = getKeys();
    double lowerBound = keys.get(0);

    assertEquals(-1.0, lowerBound);
  }

  @Test
  void findAdjacentKeysTest() {
    NavigableSet<Double> values = new TreeSet<>(getKeys());
    double distance = 3.0;
    double lowKey = values.floor(distance);
    double highKey = values.ceiling(distance);

    assertEquals(2.0, lowKey);
    assertEquals(4.0, highKey);
  }

  @Test
  void interpolateTest() {

    double lowKey = 2.0;
    double lowValue = 1500;
    double highKey = 4.0;
    double highValue = 2500;
    double x = 3.0;

    assertEquals(2000, interpolate(lowKey, lowValue, highKey, highValue, x));
  }

  private Double[] generateNumbers(int count, int range) {
    Random rand = new Random();
    List<Double> nums = new ArrayList<>();
    for (int i = 0; i < count; i++) {
      nums.add(rand.nextDouble(range + 1));
    }
    return nums.toArray(new Double[0]);
  }

  @Test
  void testAimTable() {
    DummyAimTable aimTable = new DummyAimTable();
    Double[] nums = generateNumbers(20, 20);

    for (Double num : nums) {
      AimTable.AimCharacteristic characteristic = aimTable.getSpeakerAimCharacteristic(num);
      System.out.println(num);
      System.out.println(characteristic.angle + " " + characteristic.rpm);
    }
  }
}
