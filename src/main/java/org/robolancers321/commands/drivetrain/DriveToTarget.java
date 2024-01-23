/* (C) Robolancers 2024 */
package org.robolancers321.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.robolancers321.subsystems.drivetrain.Drivetrain;

public class DriveToTarget extends Command {
  /*
   * Constants
   */

  private static final double kTranslationP = 0.0;
  private static final double kTranslationI = 0.0;
  private static final double kTranslationD = 0.0;

  private static final double kRotationP = 0.0;
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

  PIDController xController;
  PIDController zController;
  PIDController thetaController;

  public DriveToTarget(
      DoubleSupplier desiredDxSupplier,
      DoubleSupplier desiredDzSupplier,
      DoubleSupplier desiredDThetaSupplier) {
    this.drivetrain = Drivetrain.getInstance();
    this.camera = new PhotonCamera("photonvision");

    this.desiredDxSupplier = desiredDxSupplier;
    this.desiredDzSupplier = desiredDzSupplier;
    this.desiredDThetaSupplier = desiredDThetaSupplier;

    this.xController = new PIDController(kTranslationP, kTranslationI, kTranslationD);
    this.zController = new PIDController(kTranslationP, kTranslationI, kTranslationD);
    this.thetaController = new PIDController(kRotationP, kRotationI, kRotationD);

    this.addRequirements(this.drivetrain);
  }

  public DriveToTarget(double desiredDx, double desiredDz, double desiredTheta) {
    this(() -> desiredDx, () -> desiredDz, () -> desiredTheta);
  }

  @Override
  public void execute() {
    double desiredDx = this.desiredDxSupplier.getAsDouble();
    double desiredDz = this.desiredDzSupplier.getAsDouble();
    double desiredDTheta = this.desiredDThetaSupplier.getAsDouble();

    this.xController.setSetpoint(desiredDx);
    this.zController.setSetpoint(desiredDz);
    this.thetaController.setSetpoint(desiredDTheta);

    double actualDx;
    double actualDz;
    double actualDTheta;

    PhotonPipelineResult visionResults = this.camera.getLatestResult();

    SmartDashboard.putBoolean("has vision target", visionResults.hasTargets());

    if (visionResults.hasTargets()) {
      PhotonTrackedTarget bestTarget = visionResults.getBestTarget();

      Transform3d relativeCameraPosition = bestTarget.getBestCameraToTarget();

      actualDx = relativeCameraPosition.getX();
      actualDz = relativeCameraPosition.getZ();
      actualDTheta = relativeCameraPosition.getRotation().getY();
    } else {
      actualDx = desiredDx;
      actualDz = desiredDz;
      actualDTheta = desiredDTheta;
    }

    SmartDashboard.putNumber("actual target dx", actualDx);
    SmartDashboard.putNumber("actual target dz", actualDz);
    SmartDashboard.putNumber("actual target dtheta", actualDTheta);

    double xControllerOutput = this.xController.calculate(actualDx);
    double zControllerOutput = this.zController.calculate(actualDz);
    double thetaControllerOutput = this.thetaController.calculate(actualDTheta);

    SmartDashboard.putNumber("x controller output", xControllerOutput);
    SmartDashboard.putNumber("z controller output", zControllerOutput);
    SmartDashboard.putNumber("theta controller output", thetaControllerOutput);

    // TODO: check vision outputs before actually calling drive
    // this.drivetrain.drive(zControllerOutput, xControllerOutput, thetaControllerOutput, false);
  }
}
