/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.drivetrain;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;

public class Drivetrain extends SubsystemBase {
  /*
   * Singleton
   */

  private static Drivetrain instance = null;

  public static Drivetrain getInstance() {
    if (instance == null) instance = new Drivetrain();

    return instance;
  }

  /*
   * Constants
   */

  private static final String kCameraName = "MainCamera";

  private static final double kTrackWidthMeters = Units.inchesToMeters(17.5);
  private static final double kWheelBaseMeters = Units.inchesToMeters(17.5);

  private static final double kMaxSpeedMetersPerSecond = 4.0;
  private static final double kMaxOmegaRadiansPerSecond = 1.5 * Math.PI;

  public static final PathConstraints kAutoConstraints =
      new PathConstraints(4.0, 2.0, 540 * Math.PI / 180, 720 * Math.PI / 180);

  private static final SwerveDriveKinematics kSwerveKinematics =
      new SwerveDriveKinematics(
          new Translation2d(0.5 * kTrackWidthMeters, 0.5 * kWheelBaseMeters), // front left
          new Translation2d(0.5 * kTrackWidthMeters, -0.5 * kWheelBaseMeters), // front right
          new Translation2d(-0.5 * kTrackWidthMeters, -0.5 * kWheelBaseMeters), // back right
          new Translation2d(-0.5 * kTrackWidthMeters, 0.5 * kWheelBaseMeters) // back left
          );

  private static double kSecondOrderKinematicsDt = 0.2;

  private static final double kTranslationP = 2.0;
  private static final double kTranslationI = 0.0;
  private static final double kTranslationD = 0.0;

  private static final double kRotationP = 0.0;
  private static final double kRotationI = 0.0;
  private static final double kRotationD = 0.0;

  /*
   * Implementation
   */

  private SwerveModule frontLeft;
  private SwerveModule frontRight;
  private SwerveModule backRight;
  private SwerveModule backLeft;

  private AHRS gyro;

  private PhotonCamera camera;
  private SwerveDrivePoseEstimator odometry;

  private Field2d field;

  private Drivetrain() {
    this.frontLeft = SwerveModule.getFrontLeft();
    this.frontRight = SwerveModule.getFrontRight();
    this.backRight = SwerveModule.getBackRight();
    this.backLeft = SwerveModule.getBackLeft();

    this.gyro = new AHRS(SPI.Port.kMXP);
    this.zeroYaw();

    this.camera = new PhotonCamera(kCameraName);

    this.odometry =
        new SwerveDrivePoseEstimator(
            kSwerveKinematics, this.gyro.getRotation2d(), this.getModulePositions(), new Pose2d());

    this.field = new Field2d();

    SmartDashboard.putData(this.field);

    AutoBuilder.configureHolonomic(
        this::getPose,
        this::resetPose,
        this::getChassisSpeeds,
        this::drive,
        new HolonomicPathFollowerConfig(
            new PIDConstants(kTranslationP, kTranslationI, kTranslationD),
            new PIDConstants(kRotationP, kRotationI, kRotationD),
            kMaxSpeedMetersPerSecond,
            0.5 * Math.hypot(kTrackWidthMeters, kWheelBaseMeters),
            new ReplanningConfig()),
        () -> {
          var myAlliance = DriverStation.getAlliance();

          if (myAlliance.isPresent()) return myAlliance.get() == DriverStation.Alliance.Red;

          return false;
        },
        this);
  }

  public Pose2d getPose() {
    return this.odometry.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose) {
    this.odometry.resetPosition(this.gyro.getRotation2d(), this.getModulePositions(), pose);
  }

  private SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      this.frontLeft.getState(),
      this.frontRight.getState(),
      this.backRight.getState(),
      this.backLeft.getState()
    };
  }

  public ChassisSpeeds getChassisSpeeds() {
    return kSwerveKinematics.toChassisSpeeds(this.getModuleStates());
  }

  private SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      this.frontLeft.getPosition(),
      this.frontRight.getPosition(),
      this.backRight.getPosition(),
      this.backLeft.getPosition()
    };
  }

  public double getYawDeg() {
    return this.gyro.getYaw();
  }

  private void updateModules(SwerveModuleState[] states) {
    this.frontLeft.update(states[0]);
    this.frontRight.update(states[1]);
    this.backRight.update(states[2]);
    this.backLeft.update(states[3]);
  }

  private void drive(
      double desiredThrottleMPS,
      double desiredStrafeMPS,
      double desiredOmegaRadPerSec,
      boolean fieldRelative) {
    double correctedOmega =
        MathUtil.clamp(
            desiredOmegaRadPerSec, -kMaxOmegaRadiansPerSecond, kMaxOmegaRadiansPerSecond);

    // apply corrective pose logarithm
    double dt = kSecondOrderKinematicsDt;
    double angularDisplacement =
        -correctedOmega * dt; // TODO: why is this negative, maybe gyro orientation

    double sin = Math.sin(0.5 * angularDisplacement);
    double cos = Math.cos(0.5 * angularDisplacement);

    double correctedThrottle = desiredStrafeMPS * sin + desiredThrottleMPS * cos;
    double correctedStrafe = desiredStrafeMPS * cos - desiredThrottleMPS * sin;

    ChassisSpeeds speeds =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                correctedStrafe, correctedThrottle, correctedOmega, this.gyro.getRotation2d())
            : new ChassisSpeeds(correctedStrafe, correctedThrottle, correctedOmega);

    this.drive(speeds);
  }

  private void drive(ChassisSpeeds speeds) {
    SwerveModuleState[] states = kSwerveKinematics.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(states, kMaxSpeedMetersPerSecond);

    this.updateModules(states);
  }

  private void doSendables() {
    SmartDashboard.putNumber("Drive Heading", this.getYawDeg());

    Pose2d odometryPose = this.getPose();

    SmartDashboard.putNumber("Odometry Pos X (m)", odometryPose.getX());
    SmartDashboard.putNumber("Odometry Pos Y (m)", odometryPose.getY());
    SmartDashboard.putNumber("Odometry Angle (deg)", odometryPose.getRotation().getDegrees());
  }

  @Override
  public void periodic() {
    this.odometry.update(this.gyro.getRotation2d(), this.getModulePositions());

    PhotonPipelineResult latestResult = this.camera.getLatestResult();

    if (latestResult.hasTargets()) {
      MultiTargetPNPResult multiTagResult = latestResult.getMultiTagResult();

      if (multiTagResult.estimatedPose.isPresent) {
        Transform3d multiTagPoseEstimation = multiTagResult.estimatedPose.best;

        SmartDashboard.putNumber("vision dx (m)", multiTagPoseEstimation.getX());
        SmartDashboard.putNumber("vision dy (m)", multiTagPoseEstimation.getY());
        SmartDashboard.putNumber("vision dz (m)", multiTagPoseEstimation.getZ());

        SmartDashboard.putNumber("vision rz (m)", multiTagPoseEstimation.getRotation().getX());
        SmartDashboard.putNumber("vision ry (m)", multiTagPoseEstimation.getRotation().getY());
        SmartDashboard.putNumber("vision rz (m)", multiTagPoseEstimation.getRotation().getZ());

        // TODO: determine a threshold within which data can be used
        SmartDashboard.putNumber("vision error pixels", multiTagResult.estimatedPose.bestReprojErr);

        Pose2d fieldPose =
            new Pose2d(
                multiTagPoseEstimation.getX(),
                multiTagPoseEstimation.getY(),
                Rotation2d.fromRadians(multiTagPoseEstimation.getRotation().getZ()));

        this.odometry.addVisionMeasurement(fieldPose, latestResult.getTimestampSeconds());
      }
    }

    this.field.setRobotPose(this.getPose());

    this.doSendables();

    this.frontLeft.doSendables();
    this.frontRight.doSendables();
    this.backRight.doSendables();
    this.backLeft.doSendables();
  }

  public Command zeroYaw() {
    return runOnce(
        () -> {
          this.gyro.zeroYaw();
          this.gyro.setAngleAdjustment(90.0); // TODO: change this depending on navx orientation
        });
  }

  private Command stop() {
    return runOnce(() -> this.drive(0.0, 0.0, 0.0, false));
  }

  private Command feedForwardDrive(
      DoubleSupplier throttleSupplier,
      DoubleSupplier strafeSupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier fieldCentricSupplier) {
    return run(() -> {
          double desiredThrottle = throttleSupplier.getAsDouble();
          double desiredStrafe = strafeSupplier.getAsDouble();
          double desiredOmega = omegaSupplier.getAsDouble();
          boolean desiredFieldCentic = fieldCentricSupplier.getAsBoolean();

          this.drive(desiredThrottle, desiredStrafe, desiredOmega, desiredFieldCentic);
        })
        .finallyDo(() -> this.stop());
  }

  public Command teleopDrive(XboxController controller, boolean fieldCentric) {
    return this.feedForwardDrive(
        () -> 0.1 * kMaxSpeedMetersPerSecond * MathUtil.applyDeadband(controller.getLeftY(), 0.2),
        () -> -0.1 * kMaxSpeedMetersPerSecond * MathUtil.applyDeadband(controller.getLeftX(), 0.2),
        () ->
            -0.3 * kMaxOmegaRadiansPerSecond * MathUtil.applyDeadband(controller.getRightX(), 0.2),
        () -> fieldCentric);
  }

  public Command tuneModules() {
    SmartDashboard.putNumber("target module angle (deg)", 0.0);
    SmartDashboard.putNumber("target module vel (m/s)", 0.0);

    SwerveModule.initTuning();

    return run(() -> {
          double theta = SmartDashboard.getNumber("target module angle (deg)", 0.0);
          double vel = SmartDashboard.getNumber("target module vel (m/s)", 0.0);

          SwerveModuleState[] states = new SwerveModuleState[4];
          for (int i = 0; i < 4; i++)
            states[i] = new SwerveModuleState(vel, Rotation2d.fromDegrees(theta));

          this.frontLeft.tune();
          this.frontRight.tune();
          this.backRight.tune();
          this.backLeft.tune();

          this.updateModules(states);
        })
        .finallyDo(() -> this.stop());
  }

  public Command dangerouslyRunDrive(double speed) {
    return run(
        () -> {
          this.frontLeft.dangerouslyRunDrive(speed);
          this.frontRight.dangerouslyRunDrive(speed);
          this.backRight.dangerouslyRunDrive(speed);
          this.backLeft.dangerouslyRunDrive(speed);
        });
  }

  public Command dangerouslyRunTurn(double speed) {
    return run(
        () -> {
          this.frontLeft.dangerouslyRunTurn(speed);
          this.frontRight.dangerouslyRunTurn(speed);
          this.backRight.dangerouslyRunTurn(speed);
          this.backLeft.dangerouslyRunTurn(speed);
        });
  }

  public Command followPath(PathPlannerPath path) {
    return AutoBuilder.followPath(path);
  }
}
