/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.launcher;

public class AimTable {
  public static class AimCharacteristic {
    public final double angle;
    public final double rpm;

    public AimCharacteristic(double angle, double rpm) {
      this.angle = angle;
      this.rpm = rpm;
    }
  }

  private static class AimCharacteristicTableEntry {
    public final double distance;
    public final AimCharacteristic aimCharacteristic;

    public AimCharacteristicTableEntry(double distance, AimCharacteristic aimCharacteristic){
      this.distance = distance;
      this.aimCharacteristic = aimCharacteristic;
    }
  }

  // TODO: tune
  public static AimCharacteristic kRetractedAimCharacteristic = new AimCharacteristic(0.0, 0.0);
  public static AimCharacteristic kMatingAimCharacteristic = new AimCharacteristic(0.0, 0.0);
  public static AimCharacteristic kAmpAimCharacteristic = new AimCharacteristic(0.0, 0.0);

  // TODO: tune
  private static AimCharacteristicTableEntry[] table = {
    new AimCharacteristicTableEntry(0.0, new AimCharacteristic(0.0, 0.0)),
    new AimCharacteristicTableEntry(1.0, new AimCharacteristic(0.0, 0.0))
  };

  private static double interpolate(double lowKey, double lowValue, double highKey, double highValue, double x){
    double percent = (x - lowKey) / (highKey - lowKey);

    return lowKey + percent * (highValue - lowValue);
  }
  
  public static AimCharacteristic getSpeakerAimCharacteristic(double distance){
    // TODO: write this correctly (not with the tree because it has edge cases at bounds)

    return new AimCharacteristic(0.0, 0.0);
  }
}
