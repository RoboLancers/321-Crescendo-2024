/* (C) Robolancers 2024 */
package org.robolancers321.util;

public class MathUtils {

  public static boolean epsilonEquals(double a, double b, double epsilon) {
    return Math.abs(b - a) < epsilon;
  }
}
