/* (C) Robolancers 2024 */
package org.robolancers321;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class Vision {

  private static Vision instance;

  public static Vision getInstance() {

    if (instance == null) {
      instance = new Vision();
    }

    return instance;
  }

  /*
   * Constants
   */
  private static final String kCameraName = "camera";
  public static final Transform3d kRobotToCam =
      new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));

  // The layout of the AprilTags on the field
  public static final AprilTagFieldLayout kTagLayout =
      AprilTagFields.kDefaultField.loadAprilTagLayoutField();

  public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
  public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  // Probably
  private double kOutOfBoundsDistance = 4;

  /*
   * Implementation
   */
  private final PhotonCamera camera;
  private final PhotonPoseEstimator photonEstimator;
  private double lastEstTimestamp = 0;

  public Vision() {
    camera = new PhotonCamera(kCameraName);

    photonEstimator =
        new PhotonPoseEstimator(
            kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, kRobotToCam);
    photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  public EstimatedRobotPose getEstimatedGlobalPose() {
    var estimate = photonEstimator.update();

    var currentTime = camera.getLatestResult().getTimestampSeconds();

    lastEstTimestamp =
        Math.abs(currentTime - lastEstTimestamp) > 1e-5 ? currentTime : lastEstTimestamp;

    return estimate.get();
  }

  private PhotonPipelineResult getLatestResult() {
    return camera.getLatestResult();
  }

  public Matrix<N3, N1> getStdDevs(Pose3d estimatedPose) {

    var estStdDevs = kSingleTagStdDevs;
    var targets = getLatestResult().getTargets();
    double averageDistance = 0;
    int nTags = targets.size();

    if (nTags == 0) return estStdDevs;

    for (var target : targets) {
      averageDistance +=
          photonEstimator
              .getFieldTags()
              .getTagPose(target.getFiducialId())
              .get()
              .getTranslation()
              .getDistance(estimatedPose.getTranslation());
    }

    averageDistance /= nTags;

    // multiple tags woohoo we chillin yuh lets go
    if (nTags > 1) {
      estStdDevs = kMultiTagStdDevs;
      return estStdDevs;
    } else if (nTags == 1) {
      // No idea what this is but Patrick says it has something to do with ML/Stats so I'll trust
      var magicFormula = 1 + (Math.pow(averageDistance, 2) / 30);
      if (averageDistance > kOutOfBoundsDistance)
        estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
      else estStdDevs = estStdDevs.times(magicFormula);
    }

    return estStdDevs;
  }
}
