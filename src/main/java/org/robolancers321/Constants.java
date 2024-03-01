/* (C) Robolancers 2024 */
package org.robolancers321;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static final class DrivetrainConstants {
    public static final String kMainCameraName = "MainCamera";
    public static final String kNoteCameraName = "NoteCamera";

    public static final AprilTagFieldLayout kAprilTagFieldLayout =
        AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    // TODO: find this transform
    public static final Transform3d kRobotToCameraTransform =
        new Transform3d(0.34, 0, -0.48, new Rotation3d(0, 40.0 * Math.PI / 180.0, Math.PI));

    public static final double kNoteCameraMountHeight =
        Units.inchesToMeters(11.0); // rough estimate of camera height while mounted on crate
    public static final double kNoteCameraMountPitch =
        0.0; // degrees w/r to the horizontal, above horizontal is positive

    // TODO: fix this
    public static final double kTrackWidthMeters = Units.inchesToMeters(17.5);
    public static final double kWheelBaseMeters = Units.inchesToMeters(17.5);

    public static final double kMaxSpeedMetersPerSecond = 5.0;
    public static final double kMaxOmegaRadiansPerSecond = 1.5 * Math.PI;

    public static final double kMaxTeleopSpeedPercent = 1.0;
    public static final double kMaxTeleopRotationPercent = 1.0;

    public static final PathConstraints kAutoConstraints =
        new PathConstraints(4.0, 1.0, 270 * Math.PI / 180, 360 * Math.PI / 180);

    public static final SwerveDriveKinematics kSwerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(0.5 * kTrackWidthMeters, 0.5 * kWheelBaseMeters), // front left
            new Translation2d(0.5 * kTrackWidthMeters, -0.5 * kWheelBaseMeters), // front right
            new Translation2d(-0.5 * kTrackWidthMeters, -0.5 * kWheelBaseMeters), // back right
            new Translation2d(-0.5 * kTrackWidthMeters, 0.5 * kWheelBaseMeters) // back left
            );

    public static double kSecondOrderKinematicsDt = 0.2;

    public static final double kTranslationP = 2.0;
    public static final double kTranslationI = 0.0;
    public static final double kTranslationD = 0.0;

    // corrects heading during path planner
    public static final double kRotationP = 4.0;
    public static final double kRotationI = 0.0;
    public static final double kRotationD = 0.0;

    // corrects heading during teleop
    public static final double kHeadingP = 0.2; // multiply by 10 if it doesnt explode
    public static final double kHeadingI = 0.0;
    public static final double kHeadingD = 0.01; // 0.01?

    public static final double kHeadingTolerance = 1.0;
  }

  public static final class SwerveModuleConstants {
    public static final CANcoderConfiguration kCANCoderConfig = new CANcoderConfiguration();

    public static final double kWheelRadiusMeters = Units.inchesToMeters(2.0);
    public static final double kGearRatio = 6.12;
    public static final double kDrivePositionConversionFactor =
        2 * Math.PI * kWheelRadiusMeters / kGearRatio;
    public static final double kDriveVelocityConversionFactor =
        2 * Math.PI * kWheelRadiusMeters / (kGearRatio * 60.0);
    public static final double kTurnPositionConversionFactor = 7.0 / 150.0;

    public static final double kDriveP = 0.00;
    public static final double kDriveI = 0.00;
    public static final double kDriveD = 0.00;
    public static final double kDriveFF = 0.198;

    public static final double kTurnP = 0.50;
    public static final double kTurnI = 0.00;
    public static final double kTurnD = 0.005;
  }

  public static final class RetractorConstants {
    public static final int kMotorPort = 13;

    public static final boolean kInvertMotor = true;
    public static final boolean kInvertEncoder = true;

    public static final int kCurrentLimit = 40;

    public static final double kGearRatio = 360.0;

    public static final float kMinAngle = -18f;
    public static final float kMaxAngle = 160.0f;

    public static final double kP = 0.0065;
    public static final double kI = 0.000;
    public static final double kD = 0.0001;

    public static final double kS = 0.000;
    public static final double kG = 0.0155;
    public static final double kV = 0.000;

    public static final double kMaxVelocityDeg = 180.0;
    public static final double kMaxAccelerationDeg = 540.0;

    public static final double kToleranceDeg = 3.0;

    public enum RetractorSetpoint {
      kRetracted(160),
      kMating(160),
      kClearFromLauncher(50),
      kIntake(-14),
      kOuttake(20.0);

      public final double angle;

      RetractorSetpoint(double angleDeg) {
        this.angle = angleDeg;
      }
    }
  }

  public static final class SuckerConstants {
    public static final int kMotorPort = 14;

    public static final int kTouchSensorPort = 5;

    public static final boolean kInvertMotor = false;
    public static final int kCurrentLimit = 60;

    public static final double kInSpeed = 0.8;
    public static final double kOutSpeed = -0.8;

    // public static final double kFF = 0.00017;

    // public static final double kInRPM = 2000;
    // public static final double kOutRPM = -2500;
  }

  public static final class FlywheelConstants {
    public static final int kMotorPort = 17;

    public static final boolean kInvertMotor = false;
    public static final int kCurrentLimit = 60;

    public static final double kRampUpRate = 10000000; // 11200; // 12000;

    public static final double kFF = 0.00016; // TODO: this is tuned for 0.000153

    public static final double kToleranceRPM = 100.0;

    public enum FlywheelSetpoint {
      kAmp(1000.0),
      kSpeaker(5200.0);

      public final double rpm;

      FlywheelSetpoint(double rpm) {
        this.rpm = rpm;
      }
    }
  }

  public static final class IndexerConstants {
    public static final int kMotorPort = 16;
    public static final int kBeamBreakPort = 7;

    public static final boolean kInvertMotor = true;
    public static final int kCurrentLimit = 60;

    public static final double kFF = 0.000153;

    public static final double kHandoffRPM = 3000;
    public static final double kShiftForwardRPM = 600;
    public static final double kShiftBackwardRPM = -300;
    public static final double kOuttakeRPM = 3000;
  }

  public static final class PivotConstants {
    public static final int kMotorPort = 15;

    public static final boolean kInvertMotor = false;
    public static final boolean kInvertEncoder = false;
    public static final int kCurrentLimit = 40;

    public static final double kGearRatio = 360.0;

    public static final float kMinAngle = 0.0f;
    public static final float kMaxAngle = 90.0f;

    public static final double kP = 0.05;
    public static final double kI = 0.0;
    public static final double kD = 0.00;

    public static final double kS = 0.0;
    public static final double kG = 0.0;
    public static final double kV = 0.0;

    public static final double kMaxVelocityDeg = 150;
    public static final double kMaxAccelerationDeg = 600;

    public static final double kToleranceDeg = 3.0;

    public enum PivotSetpoint {
      kRetracted(15.0),
      kMating(0.0),
      kAmp(90.0),
      kSpeaker(0.0);

      public final double angle;

      PivotSetpoint(double angle) {
        this.angle = angle;
      }
    }
  }
}
