/* (C) Robolancers 2024 */
package org.robolancers321.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.robolancers321.subsystems.drivetrain.Drivetrain;

public class TuneDriveToTarget extends Command {
  /*
   * Constants
   */

  private static final double kTranslationP = 0.4;
  private static final double kTranslationI = 0.0;
  private static final double kTranslationD = 0.0;

  private static final double kRotationP = 0.3;
  private static final double kRotationI = 0.0;
  private static final double kRotationD = 0.0;

  /*
   * Implementation
   */

  Drivetrain drivetrain;
  PhotonCamera camera;

  DoubleSupplier desiredDxSupplier;
  DoubleSupplier desiredDzSupplier;
  DoubleSupplier desiredDThetaSupplier;

  PIDController positionController;
  PIDController thetaController;

  public TuneDriveToTarget(
      DoubleSupplier desiredDxSupplier,
      DoubleSupplier desiredDzSupplier,
      DoubleSupplier desiredDThetaSupplier) {
    this.drivetrain = Drivetrain.getInstance();
    this.camera = new PhotonCamera("camera");

    this.desiredDxSupplier = desiredDxSupplier;
    this.desiredDzSupplier = desiredDzSupplier;
    this.desiredDThetaSupplier = desiredDThetaSupplier;

    this.positionController = new PIDController(kTranslationP, kTranslationI, kTranslationD);
    this.thetaController = new PIDController(kRotationP, kRotationI, kRotationD);

    SmartDashboard.putNumber(
        "translation controller kp",
        SmartDashboard.getNumber("translation controller kp", kTranslationP));
    SmartDashboard.putNumber(
        "translation controller ki",
        SmartDashboard.getNumber("translation controller ki", kTranslationI));
    SmartDashboard.putNumber(
        "translation controller kd",
        SmartDashboard.getNumber("translation controller kd", kTranslationD));

    SmartDashboard.putNumber(
        "rotation controller kp", SmartDashboard.getNumber("rotation controller kp", kRotationP));
    SmartDashboard.putNumber(
        "rotation controller ki", SmartDashboard.getNumber("rotation controller ki", kRotationI));
    SmartDashboard.putNumber(
        "rotation controller kd", SmartDashboard.getNumber("rotation controller kd", kRotationD));

    this.addRequirements(this.drivetrain);
  }

  public TuneDriveToTarget(double desiredDx, double desiredDz, double desiredTheta) {
    this(() -> desiredDx, () -> desiredDz, () -> desiredTheta);
  }

  @Override
  public void execute() {
    double tunedTranslationP = SmartDashboard.getNumber("translation controller kp", kTranslationP);
    double tunedTranslationI = SmartDashboard.getNumber("translation controller ki", kTranslationI);
    double tunedTranslationD = SmartDashboard.getNumber("translation controller kd", kTranslationD);

    this.positionController.setPID(tunedTranslationP, tunedTranslationI, tunedTranslationD);

    double tunedRotationP = SmartDashboard.getNumber("rotation controller kp", kRotationP);
    double tunedRotationI = SmartDashboard.getNumber("rotation controller ki", kRotationI);
    double tunedRotationD = SmartDashboard.getNumber("rotation controller kd", kRotationD);

    this.thetaController.setPID(tunedRotationP, tunedRotationI, tunedRotationD);

    double desiredDx = this.desiredDxSupplier.getAsDouble();
    double desiredDz = this.desiredDzSupplier.getAsDouble();
    double desiredDTheta = this.desiredDThetaSupplier.getAsDouble();

    this.positionController.setSetpoint(0.0);
    this.thetaController.setSetpoint(desiredDTheta);

    double actualDx;
    double actualDz;
    double actualDTheta;

    PhotonPipelineResult visionResults = this.camera.getLatestResult();

    SmartDashboard.putBoolean("has vision target", visionResults.hasTargets());

    if (visionResults.hasTargets()) {
      PhotonTrackedTarget bestTarget = visionResults.getBestTarget();

      Transform3d relativeCameraPosition = bestTarget.getBestCameraToTarget();

      // goofy ah coordinate system
      // these are from robot looking at tag
      actualDx = -relativeCameraPosition.getY();
      actualDz = relativeCameraPosition.getX();
      actualDTheta = relativeCameraPosition.getRotation().getX() * 180.0 / Math.PI;

    } else {
      actualDx = desiredDx;
      actualDz = desiredDz;
      actualDTheta = desiredDTheta;
    }

    SmartDashboard.putNumber("actual target dx", actualDx);
    SmartDashboard.putNumber("actual target dz", actualDz);
    SmartDashboard.putNumber("actual target dtheta", actualDTheta);

    double errorX = desiredDx - actualDx;
    double errorZ = desiredDz - actualDz;
    double errorPosition = Math.hypot(errorX, errorZ);

    double positionControllerOutput =
        MathUtil.clamp(this.positionController.calculate(errorPosition), -0.3, 0.3);
    double thetaControllerOutput =
        MathUtil.clamp(this.thetaController.calculate(actualDTheta), -0.3, 0.3);

    SmartDashboard.putNumber("position controller output", positionControllerOutput);
    SmartDashboard.putNumber("theta controller output", thetaControllerOutput);

    double xComponentOfError = errorX / errorPosition;
    double zComponentOfError = errorZ / errorPosition;

    double outputX = MathUtil.clamp(xComponentOfError * positionControllerOutput, -0.3, 0.3);
    double outputZ = MathUtil.clamp(zComponentOfError * positionControllerOutput, -0.3, 0.3);

    this.drivetrain.drive(outputX, outputZ, thetaControllerOutput, false);
  }
}
