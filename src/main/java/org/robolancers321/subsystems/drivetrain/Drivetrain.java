/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.drivetrain;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.GeometryUtil;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.robolancers321.Constants.DrivetrainConstants;
import org.robolancers321.util.MathUtils;
import org.robolancers321.util.MyAlliance;

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
   * Implementation
   */

  private final SwerveModule frontLeft;
  private final SwerveModule frontRight;
  private final SwerveModule backRight;
  private final SwerveModule backLeft;

  private final AHRS gyro;

  private final PIDController headingController;

  private final PhotonCamera mainCamera;
  private final PhotonCamera noteCamera;

  private final PhotonPoseEstimator visionEstimator;

  private final SwerveDrivePoseEstimator odometry;

  private final Field2d field;
  private final Field2d visionField;

  public boolean slowMode = false;

  private Drivetrain() {
    this.frontLeft = SwerveModule.getFrontLeft();
    this.frontRight = SwerveModule.getFrontRight();
    this.backRight = SwerveModule.getBackRight();
    this.backLeft = SwerveModule.getBackLeft();

    this.gyro = new AHRS(SPI.Port.kMXP);

    this.headingController =
        new PIDController(
            DrivetrainConstants.kHeadingP,
            DrivetrainConstants.kHeadingI,
            DrivetrainConstants.kHeadingD);

    this.mainCamera = new PhotonCamera(DrivetrainConstants.kMainCameraName);
    this.noteCamera = new PhotonCamera(DrivetrainConstants.kNoteCameraName);

    this.visionEstimator =
        new PhotonPoseEstimator(
            DrivetrainConstants.kAprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            mainCamera,
            DrivetrainConstants.kRobotToCameraTransform);

    this.odometry =
        new SwerveDrivePoseEstimator(
            DrivetrainConstants.kSwerveKinematics,
            this.gyro.getRotation2d(),
            this.getModulePositions(),
            new Pose2d());

    this.field = new Field2d();
    this.visionField = new Field2d();

    this.configureGyro();
    this.configureController();
    this.configurePathPlanner();
    this.configureField();
  }

  public void setYaw(double angle) {
    this.gyro.setAngleAdjustment(-angle - this.gyro.getYaw());
  }

  public void configureGyro() {
    this.setYaw(0.0);
    // this.gyro.zeroYaw();
    // this.gyro.setAngleAdjustment(180.0); // bruh
  }

  // TODO: what the fuckkkkk why is this
  // public void setYaw(double angle) {
  //   this.gyro.zeroYaw();
  //   this.gyro.setAngleAdjustment(angle);
  // }

  private void configureController() {
    this.headingController.setPID(
        DrivetrainConstants.kHeadingP,
        DrivetrainConstants.kHeadingI,
        DrivetrainConstants.kHeadingD);
    this.headingController.enableContinuousInput(-180.0, 180.0);
    this.headingController.setTolerance(1.5);
  }

  private void configurePathPlanner() {
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::resetPose,
        this::getChassisSpeeds,
        this::driveFromSpeeds,
        new HolonomicPathFollowerConfig(
            new PIDConstants(
                // 0,0,0
                DrivetrainConstants.kTranslationP,
                DrivetrainConstants.kTranslationI,
                DrivetrainConstants.kTranslationD),
            new PIDConstants(
                // 0,0,0
                DrivetrainConstants.kRotationP,
                DrivetrainConstants.kRotationI,
                DrivetrainConstants.kRotationD),
            DrivetrainConstants.kMaxSpeedMetersPerSecond,
            0.5
                * Math.hypot(
                    DrivetrainConstants.kTrackWidthMeters, DrivetrainConstants.kWheelBaseMeters),
            new ReplanningConfig()),
        MyAlliance::isRed,
        this);
  }

  private void configureField() {
    SmartDashboard.putData("Odo Field", this.field);
    SmartDashboard.putData("Vision Field", this.visionField);

    PathPlannerLogging.setLogActivePathCallback(
        poses -> this.field.getObject("pathplanner path poses").setPoses(poses));
    PathPlannerLogging.setLogTargetPoseCallback(
        targ -> this.field.getObject("pathplanner targ pose").setPose(targ));
    PathPlannerLogging.setLogCurrentPoseCallback(
        curr -> this.field.getObject("pathplanner curr pose").setPose(curr));
  }

  public double getYawDeg() {
    return -this.gyro.getAngle();
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
    return DrivetrainConstants.kSwerveKinematics.toChassisSpeeds(this.getModuleStates());
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

  private void driveFromInput(
      double desiredThrottleMPS,
      double desiredStrafeMPS,
      double desiredOmegaRadPerSec,
      boolean fieldRelative) {
    double correctedOmega =
        MathUtil.clamp(
            desiredOmegaRadPerSec,
            -DrivetrainConstants.kMaxOmegaRadiansPerSecond,
            DrivetrainConstants.kMaxOmegaRadiansPerSecond);

    // apply corrective pose logarithm
    double dt = DrivetrainConstants.kSecondOrderKinematicsDt;
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

    this.driveFromSpeeds(speeds);
  }

  private void driveFromSpeeds(ChassisSpeeds speeds) {
    SwerveModuleState[] states = DrivetrainConstants.kSwerveKinematics.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(
        states, DrivetrainConstants.kMaxSpeedMetersPerSecond);

    this.updateModules(states);
  }

  public boolean seesTag() {
    return this.mainCamera.getLatestResult().hasTargets();
  }

  private void fuseVision() {
    Optional<EstimatedRobotPose> visionEstimate = visionEstimator.update();

    if (visionEstimate.isEmpty()) return;

    visionField.setRobotPose(visionEstimate.get().estimatedPose.toPose2d());

    this.odometry.addVisionMeasurement(
        visionEstimate.get().estimatedPose.toPose2d(), visionEstimate.get().timestampSeconds);

    SmartDashboard.putNumber("vision estimate x", visionEstimate.get().estimatedPose.getX());
    SmartDashboard.putNumber("vision estimate y", visionEstimate.get().estimatedPose.getY());
    SmartDashboard.putNumber("vision estimate z", visionEstimate.get().estimatedPose.getZ());
  }

  private Translation2d getSpeakerPosition() {
    return MyAlliance.isRed() ? new Translation2d(16.53, 5.55) : new Translation2d(0.0, 5.55);
  }

  private double getAngleToSpeaker() {
    Translation2d speakerLocation = this.getSpeakerPosition();

    double angle =
        -180
            + speakerLocation.minus(this.getPose().getTranslation()).getAngle().getDegrees()
            + this.getYawDeg();

    if (!MyAlliance.isRed()) return angle;

    double x = Math.cos(angle * Math.PI / 180);
    double y = Math.sin(angle * Math.PI / 180);

    return -Math.atan2(y, -x) * 180 / Math.PI;
  }

  public double getDistanceToSpeaker() {
    Translation2d speakerLocation = this.getSpeakerPosition();

    return this.getPose().getTranslation().getDistance(speakerLocation);
  }

  public class TrapPose {
    private double distance;
    private Pose2d pose;

    public TrapPose() {
      this.distance = Double.MAX_VALUE;
      this.pose = new Pose2d();
    }

    public TrapPose(double distance, Pose2d pose) {
      this.distance = distance;
      this.pose = pose;
    }

    public double getDistance() {
      return this.distance;
    }

    public Pose2d getPose() {
      return this.pose;
    }
  }

  public TrapPose getClosestTrapPosition() {
    Pose2d[] blueTrapPoses = {
      new Pose2d(new Translation2d(3.835, 2.29), Rotation2d.fromDegrees(-120)),
      new Pose2d(new Translation2d(3.835, 5.91), Rotation2d.fromDegrees(120)),
      new Pose2d(new Translation2d(6.97, 4.1), Rotation2d.fromDegrees(0))
    };

    Pose2d[] redTrapPoses = {
      GeometryUtil.flipFieldPose(
          new Pose2d(new Translation2d(3.835, 2.29), Rotation2d.fromDegrees(-120))),
      GeometryUtil.flipFieldPose(
          new Pose2d(new Translation2d(3.835, 5.91), Rotation2d.fromDegrees(120))),
      GeometryUtil.flipFieldPose(
          new Pose2d(new Translation2d(6.97, 4.1), Rotation2d.fromDegrees(0)))
    };

    Pose2d[] trapPosesForTeam = MyAlliance.isRed() ? redTrapPoses : blueTrapPoses;

    TrapPose closestPose = new TrapPose();

    for (int i = 0; i < trapPosesForTeam.length; i++) {
      double distance =
          trapPosesForTeam[i].getTranslation().getDistance(this.getPose().getTranslation());

      if (distance < closestPose.getDistance())
        closestPose = new TrapPose(distance, trapPosesForTeam[i]);
    }

    SmartDashboard.putNumber("Closest Trap Pose Dist", closestPose.distance);
    SmartDashboard.putString("Closest Trap Pose", closestPose.pose.toString());

    return closestPose;
  }

  public boolean seesNote() {
    return this.noteCamera.getLatestResult().hasTargets();
    // && Math.abs(this.noteCamera.getLatestResult().getBestTarget().getYaw()) < 15.0;
  }

  private double getNoteAngle() {
    PhotonPipelineResult latestResult = this.noteCamera.getLatestResult();

    if (!latestResult.hasTargets()) return 0.0;

    PhotonTrackedTarget bestTarget = latestResult.getBestTarget();

    // if (Math.abs(bestTarget.getYaw()) > 15.0) return 0.0;

    return -0.5 * bestTarget.getYaw();
  }

  // private Translation2d getRelativeNoteLocation() {
  //   PhotonPipelineResult latestResult = this.noteCamera.getLatestResult();

  //   if (!latestResult.hasTargets()) return new Translation2d();

  //   PhotonTrackedTarget bestTarget = latestResult.getBestTarget();

  //   // TODO: plus or minus mount pitch?
  //   double dz =
  //        Units.inchesToMeters(10)
  //           / Math.tan((-bestTarget.getPitch() + 24) * Math.PI / 180.0);
  //   double dx = dz * Math.tan(bestTarget.getYaw() * Math.PI / 180.0);

  //   return new Translation2d(dx, dz);
  // }

  private void initTuning() {
    SmartDashboard.putNumber(
        "drive heading kp",
        SmartDashboard.getNumber("drive heading kp", DrivetrainConstants.kHeadingP));
    SmartDashboard.putNumber(
        "drive heading ki",
        SmartDashboard.getNumber("drive heading ki", DrivetrainConstants.kHeadingI));
    SmartDashboard.putNumber(
        "drive heading kd",
        SmartDashboard.getNumber("drive heading kd", DrivetrainConstants.kHeadingD));

    SmartDashboard.putNumber("drive target heading", this.getYawDeg());
  }

  private void tune() {
    double tunedHeadingP =
        SmartDashboard.getNumber("drive heading kp", DrivetrainConstants.kHeadingP);
    double tunedHeadingI =
        SmartDashboard.getNumber("drive heading ki", DrivetrainConstants.kHeadingI);
    double tunedHeadingD =
        SmartDashboard.getNumber("drive heading kd", DrivetrainConstants.kHeadingD);

    this.headingController.setPID(tunedHeadingP, tunedHeadingI, tunedHeadingD);

    // double targetHeading = SmartDashboard.getNumber("drive target heading", this.getYawDeg());

    // this.headingController.setSetpoint(targetHeading);

    // double headingControllerOutput = -this.headingController.calculate(this.getYawDeg());

    // this.drive(0.0, 0.0, headingControllerOutput, true);
  }

  private void doSendables() {
    SmartDashboard.putNumber("drive heading (deg)", this.getYawDeg());
    SmartDashboard.putNumber("internal navx sensor yaw", this.gyro.getYaw());
    SmartDashboard.putNumber("internal navx angle adjustment", this.gyro.getAngleAdjustment());

    Pose2d odometryPose = this.getPose();

    SmartDashboard.putNumber("odomFetry pos x (m)", odometryPose.getX());
    SmartDashboard.putNumber("odometry pos y (m)", odometryPose.getY());
    SmartDashboard.putNumber("odometry angle (deg)", odometryPose.getRotation().getDegrees());

    SmartDashboard.putNumber("distance to speaker", this.getDistanceToSpeaker());
    SmartDashboard.putNumber("angle to speaker", this.getAngleToSpeaker());

    SmartDashboard.putBoolean("sees note", this.seesNote());
    SmartDashboard.putNumber("angle to note", this.getNoteAngle());

    SmartDashboard.putNumber("chassis speeds x", this.getChassisSpeeds().vxMetersPerSecond);
    SmartDashboard.putNumber("chassis speeds y", this.getChassisSpeeds().vyMetersPerSecond);

    // Translation2d notePose = this.getRelativeNoteLocation();

    // SmartDashboard.putNumber("note pose x", notePose.getX());
    // SmartDashboard.putNumber("note pose y", notePose.getY());
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
    return runOnce(() -> this.driveFromInput(0.0, 0.0, 0.0, false));
  }

  public Command teleopDrive(XboxController controller, boolean fieldCentric) {
    return run(() -> {
          double multiplier = controller.getRightBumper() ? 0.4 : 1.0;

          double omega =
              -DrivetrainConstants.kMaxTeleopRotationPercent
                  * DrivetrainConstants.kMaxOmegaRadiansPerSecond
                  * MathUtil.applyDeadband(MathUtils.squareKeepSign(controller.getRightX()), 0.05)
                  * multiplier;

          // TODO: uncomment for aim assist

          // double headingControllerOutput =
          //     -this.headingController.calculate(this.getNoteAngle(), 0.0);

          // if (Math.abs(this.getNoteAngle()) > DrivetrainConstants.kHeadingTolerance)
          //   omega += 0.5 * headingControllerOutput;

          Translation2d strafeVec =
              new Translation2d(
                      DrivetrainConstants.kMaxTeleopSpeedPercent
                          * DrivetrainConstants.kMaxSpeedMetersPerSecond
                          * MathUtil.applyDeadband(
                              -MathUtils.squareKeepSign(controller.getLeftY()), 0.05)
                          * multiplier,
                      DrivetrainConstants.kMaxTeleopSpeedPercent
                          * DrivetrainConstants.kMaxSpeedMetersPerSecond
                          * MathUtil.applyDeadband(
                              MathUtils.squareKeepSign(controller.getLeftX()), 0.05)
                          * multiplier)
                  .rotateBy(Rotation2d.fromDegrees(90.0));

          this.driveFromInput(strafeVec.getX(), strafeVec.getY(), omega, true);
        })
        .finallyDo(this::stop);
  }

  public Command driveCommand(
      DoubleSupplier throttleSupplier,
      DoubleSupplier strafeSupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier fieldRelativeSupplier) {
    return run(
        () ->
            this.driveFromInput(
                throttleSupplier.getAsDouble(),
                strafeSupplier.getAsDouble(),
                omegaSupplier.getAsDouble(),
                fieldRelativeSupplier.getAsBoolean()));
  }

  public Command driveCommand(double throttle, double strafe, double omega, boolean fieldRelative) {
    return this.driveCommand(() -> throttle, () -> strafe, () -> omega, () -> fieldRelative);
  }

  public Command turnToAngle(double angle) {
    return runOnce(() -> this.headingController.setSetpoint(angle))
        .andThen(
            run(() -> {
                  double headingControllerOutput =
                      -this.headingController.calculate(this.getYawDeg(), angle);

                  this.driveFromInput(0.0, 0.0, headingControllerOutput, true);
                })
                .until(this.headingController::atSetpoint));
  }

  public Command driveIntoNote() {
    return run(() -> {
          double headingControllerOutput =
              -this.headingController.calculate(this.getNoteAngle(), 0.0);

          this.driveFromInput(0.0, 1.5, headingControllerOutput, false);
        })
        .until(() -> !this.seesNote())
        .withTimeout(2.5);
  }

  private Command turnToAngle(DoubleSupplier angleSupplier) {
    return runOnce(() -> this.headingController.setSetpoint(angleSupplier.getAsDouble()))
        .andThen(
            run(
                () -> {
                  double headingControllerOutput =
                      -this.headingController.calculate(angleSupplier.getAsDouble(), 0.0);

                  this.driveFromInput(0.0, 0.0, headingControllerOutput, true);
                }))
        .until(this.headingController::atSetpoint);
  }

  public Command turnToNote() {
    return this.turnToAngle(this::getNoteAngle);
  }

  public Command turnToSpeaker() {
    return this.turnToAngle(this::getAngleToSpeaker).withTimeout(1.0);
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

    return run(this::tune);
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

  public Command pathfindToTrap() {
    return AutoBuilder.pathfindToPose(
        this.getClosestTrapPosition().pose, DrivetrainConstants.kAutoConstraints);
  }

  // TODO: distance filter, same as trap
  public Command alignToAmp() {
    return AutoBuilder.pathfindThenFollowPath(
        PathPlannerPath.fromPathFile("AmpAlign"), DrivetrainConstants.kAutoConstraints);
  }
}
