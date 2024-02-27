/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.drivetrain;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
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
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

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

  private static final String kMainCameraName = "MainCamera";
  private static final String kNoteCameraName = "NoteCamera";

  private static final AprilTagFieldLayout kAprilTagFieldLayout =
      AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  // TODO: find this transform
  private static final Transform3d kRobotToCameraTransform =
      new Transform3d(0.34, 0, -0.48, new Rotation3d(0, 40.0 * Math.PI / 180.0, Math.PI));

  private static final double kNoteCameraMountHeight =
      Units.inchesToMeters(11.0); // rough estimate of camera height while mounted on crate
  private static final double kNoteCameraMountPitch =
      0.0; // degrees w/r to the horizontal, above horizontal is positive

  // TODO: fix this
  private static final double kTrackWidthMeters = Units.inchesToMeters(17.5);
  private static final double kWheelBaseMeters = Units.inchesToMeters(17.5);

  private static final double kMaxSpeedMetersPerSecond = 4.0;
  private static final double kMaxOmegaRadiansPerSecond = 1.5 * Math.PI;

  private static final double kMaxTeleopSpeedPercent = 1.0;
  private static final double kMaxTeleopRotationPercent = 1.0;

  public static final PathConstraints kAutoConstraints =
      new PathConstraints(4.0, 1.0, 270 * Math.PI / 180, 360 * Math.PI / 180);

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

  // corrects heading during path planner
  private static final double kRotationP = 4.0;
  private static final double kRotationI = 0.0;
  private static final double kRotationD = 0.0;

  // corrects heading during teleop
  private static final double kHeadingP = 0; // 0.25;
  private static final double kHeadingI = 0.0;
  private static final double kHeadingD = 0.0;

  /*
   * Implementation
   */

  private final SwerveModule frontLeft;
  private final SwerveModule frontRight;
  private final SwerveModule backRight;
  private final SwerveModule backLeft;

  private final AHRS gyro;

  private double lastCommandedOmega;
  private final PIDController headingController;

  private final PhotonCamera mainCamera;
  // private final PhotonCamera noteCamera;

  private final PhotonPoseEstimator visionEstimator;

  private final SwerveDrivePoseEstimator odometry;

  private final Field2d field;

  private Drivetrain() {
    this.frontLeft = SwerveModule.getFrontLeft();
    this.frontRight = SwerveModule.getFrontRight();
    this.backRight = SwerveModule.getBackRight();
    this.backLeft = SwerveModule.getBackLeft();

    this.gyro = new AHRS(SPI.Port.kMXP);

    this.headingController = new PIDController(kHeadingP, kHeadingI, kHeadingD);

    this.mainCamera = new PhotonCamera(kMainCameraName);
    // this.noteCamera = new PhotonCamera(kNoteCameraName);

    this.visionEstimator =
        new PhotonPoseEstimator(
            kAprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            mainCamera,
            kRobotToCameraTransform);

    this.odometry =
        new SwerveDrivePoseEstimator(
            kSwerveKinematics, this.gyro.getRotation2d(), this.getModulePositions(), new Pose2d());

    this.field = new Field2d();

    this.configureGyro();
    this.configureController();
    this.configurePathPlanner();
    this.configureField();
  }

  public void configureGyro() {
    this.gyro.zeroYaw();
    this.gyro.setAngleAdjustment(0.0); // TODO: change this depending on navx orientation
    // this.gyro.setAngleAdjustment(90.0); // TODO: change this depending on navx orientation
    this.headingController.setSetpoint(0.0);
  }

  private void configureController() {
    this.headingController.setPID(kHeadingP, kHeadingI, kHeadingD);
    this.headingController.enableContinuousInput(-180.0, 180.0);
    this.headingController.setTolerance(3.0);
  }

  private void configurePathPlanner() {
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

  private void configureField() {
    SmartDashboard.putData(this.field);

    PathPlannerLogging.setLogActivePathCallback(
        poses -> this.field.getObject("pathplanner path poses").setPoses(poses));
    PathPlannerLogging.setLogTargetPoseCallback(
        targ -> this.field.getObject("pathplanner targ pose").setPose(targ));
    PathPlannerLogging.setLogCurrentPoseCallback(
        curr -> this.field.getObject("pathplanner curr pose").setPose(curr));
  }

  public double getYawDeg() {
    return this.gyro.getYaw();
  }

  public Pose2d getPose() {
    return this.odometry.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose) {
    this.odometry.resetPosition(
        Rotation2d.fromDegrees(this.getYawDeg()), // .plus(Rotation2d.fromDegrees(180)),
        this.getModulePositions(),
        pose);
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

  private void fuseVision() {
    Optional<EstimatedRobotPose> visionEstimate = visionEstimator.update();

    if (visionEstimate.isEmpty()) return;

    this.odometry.addVisionMeasurement(
        visionEstimate.get().estimatedPose.toPose2d(), visionEstimate.get().timestampSeconds);
  }

  // private Translation2d getRelativeNoteLocation() {
  //   PhotonPipelineResult latestResult = this.noteCamera.getLatestResult();

  //   if (!latestResult.hasTargets()) return new Translation2d();

  //   PhotonTrackedTarget bestTarget = latestResult.getBestTarget();

  //   // TODO: plus or minus mount pitch?
  //   double dz =
  //       kNoteCameraMountHeight
  //           / Math.tan((-bestTarget.getPitch() + kNoteCameraMountPitch) * Math.PI / 180.0);
  //   double dx = dz * Math.tan(bestTarget.getYaw() * Math.PI / 180.0);

  //   return new Translation2d(dx, dz);
  // }

  private void initTuning() {
    // SmartDashboard.putNumber("robot to camera dx", SmartDashboard.getNumber("robot to camera dx",
    // kRobotToCameraTransform.getX()));
    // SmartDashboard.putNumber("robot to camera dy", SmartDashboard.getNumber("robot to camera dy",
    // kRobotToCameraTransform.getY()));
    // SmartDashboard.putNumber("robot to camera dz", SmartDashboard.getNumber("robot to camera dz",
    // kRobotToCameraTransform.getZ()));

    // SmartDashboard.putNumber("robot to camera rx", SmartDashboard.getNumber("robot to camera rx",
    // kRobotToCameraTransform.getRotation().getX()));
    // SmartDashboard.putNumber("robot to camera ry", SmartDashboard.getNumber("robot to camera ry",
    // kRobotToCameraTransform.getRotation().getY()));
    // SmartDashboard.putNumber("robot to camera rz", SmartDashboard.getNumber("robot to camera rz",
    // kRobotToCameraTransform.getRotation().getZ()));

    SmartDashboard.putNumber(
        "drive heading kp", SmartDashboard.getNumber("drive heading kp", kHeadingP));
    SmartDashboard.putNumber(
        "drive heading ki", SmartDashboard.getNumber("drive heading ki", kHeadingI));
    SmartDashboard.putNumber(
        "drive heading kd", SmartDashboard.getNumber("drive heading kd", kHeadingD));

    SmartDashboard.putNumber("drive target heading", this.getYawDeg());
  }

  private void tune() {
    // Transform3d tunedRobotToCameraTransform = new Transform3d(
    //   SmartDashboard.getNumber("robot to camera dx", kRobotToCameraTransform.getX()),
    //   SmartDashboard.getNumber("robot to camera dy", kRobotToCameraTransform.getY()),
    //   SmartDashboard.getNumber("robot to camera dz", kRobotToCameraTransform.getZ()),
    //   new Rotation3d(
    //     SmartDashboard.getNumber("robot to camera rx",
    // kRobotToCameraTransform.getRotation().getX()),
    //     SmartDashboard.getNumber("robot to camera ry",
    // kRobotToCameraTransform.getRotation().getY()),
    //     SmartDashboard.getNumber("robot to camera rz",
    // kRobotToCameraTransform.getRotation().getZ())
    //   )
    // );

    // this.visionEstimator.setRobotToCameraTransform(tunedRobotToCameraTransform);

    double tunedHeadingP = SmartDashboard.getNumber("drive heading kp", kHeadingP);
    double tunedHeadingI = SmartDashboard.getNumber("drive heading ki", kHeadingI);
    double tunedHeadingD = SmartDashboard.getNumber("drive heading kd", kHeadingD);

    this.headingController.setPID(tunedHeadingP, tunedHeadingI, tunedHeadingD);

    double targetHeading = SmartDashboard.getNumber("drive target heading", this.getYawDeg());

    this.headingController.setSetpoint(targetHeading);

    this.drive(0.0, 0.0, -this.headingController.calculate(this.getYawDeg()), true);
  }

  private void doSendables() {
    SmartDashboard.putNumber("drive heading (deg)", this.getYawDeg());

    Pose2d odometryPose = this.getPose();

    SmartDashboard.putNumber("odometry pos x (m)", odometryPose.getX());
    SmartDashboard.putNumber("odometry pos y (m)", odometryPose.getY());
    SmartDashboard.putNumber("odometry angle (deg)", odometryPose.getRotation().getDegrees());

    // Translation2d note = this.getRelativeNoteLocation();

    // SmartDashboard.putNumber("note x offset", note.getX());
    // SmartDashboard.putNumber("note z offset", note.getY());
  }

  @Override
  public void periodic() {
    this.odometry.update(this.gyro.getRotation2d(), this.getModulePositions());
    this.fuseVision();

    this.field.setRobotPose(this.getPose());

    this.doSendables();

    this.frontLeft.doSendables();
    this.frontRight.doSendables();
    this.backRight.doSendables();
    this.backLeft.doSendables();
  }

  public Command zeroYaw() {
    return runOnce(this::configureGyro);
  }

  public Command stop() {
    return runOnce(() -> this.drive(0.0, 0.0, 0.0, false));
  }

  public Command teleopDrive(XboxController controller, boolean fieldCentric) {
    this.headingController.setSetpoint(this.getYawDeg());
    this.lastCommandedOmega = 0.0;

    return run(() -> {
          double omega =
              -kMaxTeleopRotationPercent
                  * kMaxOmegaRadiansPerSecond
                  * MathUtil.applyDeadband(controller.getRightX(), 0.05);

          double turnOutput;

          if (omega == 0) {
            if (this.lastCommandedOmega != 0) {
              // the angular momentum of the robot will keep it turning for a moment after you stop
              // so we set the heading setpoint to the current angle plus a portion of its current
              // velocity
              this.headingController.setSetpoint(
                  this.getYawDeg() + 0.1 * this.gyro.getRate() * 180.0 / Math.PI);
            }

            turnOutput = -this.headingController.calculate(this.getYawDeg());
          } else {
            turnOutput = omega;
          }

          this.lastCommandedOmega = omega;

          SmartDashboard.putNumber(
              "drive heading controller setpoint", this.headingController.getSetpoint());

          Translation2d strafeVec =
              new Translation2d(
                      kMaxTeleopSpeedPercent
                          * kMaxSpeedMetersPerSecond
                          * MathUtil.applyDeadband(-controller.getLeftY(), 0.05),
                      kMaxTeleopSpeedPercent
                          * kMaxSpeedMetersPerSecond
                          * MathUtil.applyDeadband(controller.getLeftX(), 0.05))
                  .rotateBy(Rotation2d.fromDegrees(90.0));

          this.drive(strafeVec.getX(), strafeVec.getY(), turnOutput, true);
        })
        .finallyDo(this::stop);

    // return run(
    //     () ->
    //         this.drive(
    //             kMaxTeleopSpeedPercent
    //                 * kMaxSpeedMetersPerSecond
    //                 * MathUtil.applyDeadband(controller.getLeftY(), 0.02),
    //             -kMaxTeleopSpeedPercent
    //                 * kMaxSpeedMetersPerSecond
    //                 * MathUtil.applyDeadband(controller.getLeftX(), 0.02),
    //             -kMaxTeleopRotationPercent
    //                 * kMaxOmegaRadiansPerSecond
    //                 * MathUtil.applyDeadband(controller.getRightX(), 0.02),
    //             fieldCentric));
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

  public Command tuneController(XboxController controller) {
    this.initTuning();
    this.headingController.setSetpoint(this.getYawDeg());
    this.lastCommandedOmega = 0.0;

    return run(() -> {
          // TODO: multiply by some shit
          double omega =
              -kMaxTeleopRotationPercent
                  * kMaxOmegaRadiansPerSecond
                  * MathUtil.applyDeadband(controller.getRightX(), 0.05);

          double turnOutput;

          if (omega == 0) {
            if (this.lastCommandedOmega != 0) {
              // the angular momentum of the robot will keep it turning for a moment after you stop
              // so we set the heading setpoint to the current angle plus a portion of its current
              // velocity
              this.headingController.setSetpoint(
                  this.getYawDeg() + 0.1 * this.gyro.getRate() * 180.0 / Math.PI);
            }

            turnOutput = -this.headingController.calculate(this.getYawDeg());
          } else {
            turnOutput = omega;
          }

          this.lastCommandedOmega = omega;

          SmartDashboard.putNumber(
              "drive heading controller setpoint", this.headingController.getSetpoint());

          this.tune();

          this.drive(0.0, 0.0, turnOutput, true);
        })
        .finallyDo(this::stop);
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
