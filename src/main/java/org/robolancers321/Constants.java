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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static final class DrivetrainConstants {
    public static final String kMainCameraName = "MainCamera";
    public static final String kNoteCameraName = "NoteCamera";

    public static final AprilTagFieldLayout kAprilTagFieldLayout =
        AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    public static final Transform3d kRobotToCameraTransform =
        new Transform3d(0.34, 0, -0.48, new Rotation3d(0, 40.0 * Math.PI / 180.0, Math.PI));

    public static final double kNoteCameraMountHeight =
        Units.inchesToMeters(11.0); // rough estimate of camera height while mounted on crate
    public static final double kNoteCameraMountPitch =
        0.0; // degrees w/r to the horizontal, above horizontal is positive

    public static final double kTrackWidthMeters = Units.inchesToMeters(22.0);
    public static final double kWheelBaseMeters = Units.inchesToMeters(26.0);

    public static final double kMaxSpeedMetersPerSecond = 5.0;
    public static final double kMaxOmegaRadiansPerSecond = 1.5 * Math.PI;

    public static final double kMaxTeleopSpeedPercent = 1.0;
    public static final double kMaxTeleopRotationPercent = 1.0;

    public static final PathConstraints kAutoConstraints =
        new PathConstraints(4.0, 3.0, 270 * Math.PI / 180, 360 * Math.PI / 180);

    public static final SwerveDriveKinematics kSwerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(0.5 * kTrackWidthMeters, 0.5 * kWheelBaseMeters), // front left
            new Translation2d(0.5 * kTrackWidthMeters, -0.5 * kWheelBaseMeters), // front right
            new Translation2d(-0.5 * kTrackWidthMeters, -0.5 * kWheelBaseMeters), // back right
            new Translation2d(-0.5 * kTrackWidthMeters, 0.5 * kWheelBaseMeters) // back left
            );

    public static double kSecondOrderKinematicsDt = 0.2;

    public static final double kTranslationP = 1.15; // 4.0
    public static final double kTranslationI = 0.0;
    public static final double kTranslationD = 0.0;

    // corrects heading during path planner
    public static final double kRotationP = 2.16; // 8.5
    public static final double kRotationI = 0.0;
    public static final double kRotationD = 0.0;

    // corrects heading during teleop
    public static final double kHeadingP = 0.2; // multiply by 10 if it doesnt explode
    public static final double kHeadingI = 0.0;
    public static final double kHeadingD = 0.01; // 0.01?

    public static final double kHeadingTolerance = 1.5;
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
    public static final double kDriveFF = 0.198; // 0.198

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

    public static final float kMinAngle = -7f;
    public static final float kMaxAngle = 182.0f;

    public static final double kP = 0.008; //0.0065
    public static final double kI = 0.000;
    public static final double kD = 0.0001;

    public static final double kS = 0.000;
    public static final double kG = 0.037; // 0.04? // 0.0155;
    public static final double kV = 0.000;

    public static final double kMaxVelocityDeg = 120; // 400.0; //180
    public static final double kMaxAccelerationDeg = 500.0; //2000

    public static final double kToleranceDeg = 5.0;

    public enum RetractorSetpoint {
      kRetracted(182),                                                                                                                                                                                                                                                          
      kMating(173),
      kIntake(-6),
      kOuttake(45.0);

      public final double angle;

      RetractorSetpoint(double angleDeg) {
        this.angle = angleDeg;
      }
    }
  }

  public static final class SuckerConstants {
    public static final int kMotorPort = 14;

    public static final int kTouchSensorPort = 5;

    public static final boolean kInvertMotor = true;
    public static final int kCurrentLimit = 40;

    public static final double kInSpeed = 1.0;
    public static final double kOutSpeed = -1.0;

    // public static final double kFF = 0.00017;

    // public static final double kInRPM = 2000;
    // public static final double kOutRPM = -2500;
  }

  public static final class FlywheelConstants {
    public static final int kMotorPort = 17;

    public static final boolean kInvertMotor = false;
    public static final int kCurrentLimit = 75;

    public static final double kRampUpRate =
        10000000; // effectively infinite ramp up, keeping this for the infrastructure

    public static final double kFF = 0.0001502;
    public static final double kToleranceRPM = 60.0;

    public enum FlywheelSetpoint {
      kAcceptHandoff(150), // this is super finicky
      kShiftForward(50),
      kShiftBackwardFast(-4000),
      kShiftBackwardSlow(-1000),
      kAmp(1500.0),
      kSpeaker(2500.0);

      public final double rpm;

      FlywheelSetpoint(double rpm) {
        this.rpm = rpm;
      }
    }
  }

  public static final class IndexerConstants {
    public static final int kMotorPort = 16;

    public static final int kEntranceBeamBreakPort = 7;
    public static final int kExitBeamBreakPort = 9;

    public static final boolean kInvertMotor = true;
    public static final int kCurrentLimit = 40;

    public static final double kFF = 0.000153;

    public static final double kHandoffRPM = 2000;
    public static final double kShiftBackFromExitRPM = -600;
    public static final double kShiftBackToEntranceRPM = -100;
    public static final double kShiftForwardFromEntranceRPM = 200;
    public static final double kOuttakeRPM = 3000;
  }

  public static final class PivotConstants {
    public static final int kMotorPort = 15;

    public static final boolean kInvertMotor = false;
    public static final boolean kInvertEncoder = false;
    public static final int kCurrentLimit = 40;

    public static final double kGearRatio = 360.0;

    public static final float kMinAngle = -19f;
    public static final float kMaxAngle = 75f;

    public static final double kP = 0.04; // 0.03
    public static final double kI = 0.0;
    public static final double kD = 0.0; // 0.02;

    public static final double kS = 0.0;
    public static final double kG = 0.023;
    public static final double kV = 0.0; // 0.35

    public static final double kMaxVelocityDeg = 160;
    public static final double kMaxAccelerationDeg = 1500;
    public static TrapezoidProfile.Constraints kProfileConstraints =
        new Constraints(kMaxVelocityDeg, kMaxAccelerationDeg);

    public static final double kToleranceDeg = 1.0;

    public enum PivotSetpoint {
      kRetracted(-13.0),
      kMating(-19.0),
      kAmp(75.0),
      kSpeaker(-16.0);

      public final double angle;

      PivotSetpoint(double angle) {
        this.angle = angle;
      }
    }
  }

  public static final class AimConstants {
    public static final double kMinDistance = 1.4;
    public static final double kMaxDistance = 5.4;

    public static final class PivotAngleCoefficients {
      public static final double kA = -22.6139;
      public static final double kB = -0.826057;
      public static final double kC = 1.66209;
      public static final double kD = 12.2049;
    }

    public static final class FlywheelRPMCoefficients {
      public static final double kA = -1004.55;
      public static final double kB = -0.884096;
      public static final double kC = 2.93695;
      public static final double kD = 3477.5;
    }
  }
}
