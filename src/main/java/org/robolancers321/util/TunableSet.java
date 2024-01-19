/* (C) Robolancers 2024 */
package org.robolancers321.util;

import static edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.*;

public class TunableSet {
  public static class Tunable {
    public static int tune(String key, int defaultValue) {
      var entry = getEntry(key);
      if (!entry.exists()) putNumber(key, defaultValue);
      return (int) entry.getInteger(defaultValue);
    }

    public static double tune(String key, double defaultValue) {
      var entry = getEntry(key);
      if (!entry.exists()) putNumber(key, defaultValue);
      return entry.getDouble(defaultValue);
    }

    public static String tune(String key, String defaultValue) {
      var entry = getEntry(key);
      if (!entry.exists()) putString(key, defaultValue);
      return entry.getString(defaultValue);
    }

    public static boolean tune(String key, boolean defaultValue) {
      var entry = getEntry(key);
      if (!entry.exists()) putBoolean(key, defaultValue);
      return entry.getBoolean(defaultValue);
    }
  }

  private String prefix;

  public TunableSet(String prefix) {
    this.prefix = prefix;
  }

  // !TODO Add prefixes
  public int tune(String key, int defaultValue) {
    var entry = getEntry(prefix + " " + key);
    if (!entry.exists()) putNumber(key, defaultValue);
    return (int) entry.getInteger(defaultValue);
  }

  public double tune(String key, double defaultValue) {
    var entry = getEntry(prefix + " " + key);
    if (!entry.exists()) putNumber(key, defaultValue);
    return entry.getDouble(defaultValue);
  }

  public String tune(String key, String defaultValue) {
    var entry = getEntry(prefix + " " + key);
    if (!entry.exists()) putString(key, defaultValue);
    return entry.getString(defaultValue);
  }

  public boolean tune(String key, boolean defaultValue) {
    var entry = getEntry(prefix + " " + key);
    if (!entry.exists()) putBoolean(key, defaultValue);
    return entry.getBoolean(defaultValue);
  }
}
