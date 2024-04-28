/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.launcher;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AimTable {
  // unit test
  public static void main(String[] args) {
    for (double x = 0.0; x < 10.0; x += 0.2) {
      double pivotAngle = interpolatePivotAngle(x);
      double flywheelRPM = interpolateFlywheelRPM(x);

      System.out.println(
          "For distance "
              + x
              + ": \t\tpivot angle = "
              + pivotAngle
              + ", \t\tflywheel rpm = "
              + flywheelRPM);
    }

    System.out.println(
        "For distance NaN: \t\tpivot angle = "
            + interpolatePivotAngle(Double.NaN)
            + ", \t\tflywheel rpm = "
            + interpolateFlywheelRPM(Double.NaN));
    System.out.println(
        "For distance +Inf: \t\tpivot angle = "
            + interpolatePivotAngle(Double.POSITIVE_INFINITY)
            + ", \t\tflywheel rpm = "
            + interpolateFlywheelRPM(Double.POSITIVE_INFINITY));
    System.out.println(
        "For distance -Inf: \t\tpivot angle = "
            + interpolatePivotAngle(Double.NEGATIVE_INFINITY)
            + ", \t\tflywheel rpm = "
            + interpolateFlywheelRPM(Double.NEGATIVE_INFINITY));
    System.out.println(
        "For distance MaxVal: \t\tpivot angle = "
            + interpolatePivotAngle(Double.MAX_VALUE)
            + ", \t\tflywheel rpm = "
            + interpolateFlywheelRPM(Double.MAX_VALUE));
    System.out.println(
        "For distance MinVal: \t\tpivot angle = "
            + interpolatePivotAngle(Double.MIN_VALUE)
            + ", \t\tflywheel rpm = "
            + interpolateFlywheelRPM(Double.MIN_VALUE));
  }

  public static double interpolatePivotAngle(double distance) {
    return SmartDashboard.getNumber("tuning pivot angle", 0.0);

    // if (Double.isNaN(distance) || distance < AimConstants.kMinDistance)
    //   return interpolatePivotAngle(AimConstants.kMinDistance);

    // if (distance > AimConstants.kMaxDistance)
    //   return interpolatePivotAngle(AimConstants.kMaxDistance);

    // return AimConstants.PivotAngleCoefficients.kA
    //         * Math.atan(
    //             AimConstants.PivotAngleCoefficients.kB * distance
    //                 + AimConstants.PivotAngleCoefficients.kC)
    //     + AimConstants.PivotAngleCoefficients.kD;
  }

  public static double interpolateFlywheelRPM(double distance) {
    return SmartDashboard.getNumber("tuning flywheel rpm", 0.0);

    // if (Double.isNaN(distance) || distance < AimConstants.kMinDistance)
    //   return interpolateFlywheelRPM(AimConstants.kMinDistance);

    // if (distance > AimConstants.kMaxDistance)
    //   return interpolateFlywheelRPM(AimConstants.kMaxDistance);

    // return 2400;

    // return AimConstants.FlywheelRPMCoefficients.kA
    //         * Math.atan(
    //             AimConstants.FlywheelRPMCoefficients.kB * distance
    //                 + AimConstants.FlywheelRPMCoefficients.kC)
    //     + AimConstants.FlywheelRPMCoefficients.kD;
  }
}
