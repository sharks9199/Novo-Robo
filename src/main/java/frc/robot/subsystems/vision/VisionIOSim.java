package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import java.util.Random;
import java.util.function.Supplier;

/**
 * Simulated Limelight.
 *
 * <p>Given the robot's "true" pose, it determines which AprilTags the camera would see (range,
 * field of view and tag facing direction) and produces noisy MegaTag 1 / MegaTag 2 poses, at a
 * limited frame rate and with latency, just like {@link VisionIOReal} would deliver.
 *
 * <p>Limitations: tags are never occluded by other field elements, and the model is 2D (camera
 * height, pitch and tag height are ignored).
 */
public class VisionIOSim implements VisionIO {

  // ---------------------------------------------------------------------------
  // Camera model
  // ---------------------------------------------------------------------------

  /** Camera pose relative to the robot center (default: 0.3 m forward, facing forward). */
  private static final Transform2d ROBOT_TO_CAMERA =
      new Transform2d(new Translation2d(0.3, 0.0), Rotation2d.kZero);

  /** Horizontal field of view, in degrees. */
  private static final double HORIZONTAL_FOV_DEG = 75.0;

  /** Tags viewed at more than this angle off their normal are considered unreadable, in degrees. */
  private static final double MAX_TAG_VIEW_ANGLE_DEG = 70.0;

  /** Camera frame rate, in Hz. */
  private static final double FRAME_RATE_HZ = 30.0;

  /** Pipeline plus capture latency, in seconds. */
  private static final double LATENCY_SECONDS = 0.030;

  // ---------------------------------------------------------------------------
  // Measurement noise
  // ---------------------------------------------------------------------------

  /** Position noise standard deviation per meter of tag distance (MegaTag 2). */
  private static final double MT2_XY_NOISE_PER_METER = 0.02;

  /** Position noise standard deviation per meter of tag distance (MegaTag 1). */
  private static final double MT1_XY_NOISE_PER_METER = 0.05;

  /** Heading noise standard deviation of MegaTag 1, in degrees. */
  private static final double MT1_HEADING_NOISE_DEG = 2.0;

  /** Ambiguity reported for a single, well-seen tag. */
  private static final double SINGLE_TAG_AMBIGUITY = 0.05;

  // ---------------------------------------------------------------------------
  // State
  // ---------------------------------------------------------------------------

  private final AprilTagFieldLayout layout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  private final Supplier<Pose2d> truePoseSupplier;
  private final Random random = new Random(0);

  private double lastFrameTime = -1.0;

  // Values written by Vision, kept for debugging (MegaTag 2 uses them on a real Limelight).

  @SuppressWarnings("unused")
  private int pipeline = 0;
  @SuppressWarnings("unused")
  private double robotYawDeg = 0.0;
  @SuppressWarnings("unused")
  private double robotYawRateDegPerSec = 0.0;

  /**
   * Creates a simulated camera.
   *
   * @param truePoseSupplier supplies the robot's real pose in the simulated world, e.g. {@code
   *     drive::getPose}
   */
  public VisionIOSim(Supplier<Pose2d> truePoseSupplier) {
    this.truePoseSupplier = truePoseSupplier;
  }

  @Override
  public void definirPipeline(int pipeline) {
    this.pipeline = pipeline;
  }

  @Override
  public void definirOrientacao(double yawGraus, double yawRateGrausPorSegundo) {
    robotYawDeg = yawGraus;
    robotYawRateDegPerSec = yawRateGrausPorSegundo;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    double now = Timer.getFPGATimestamp();

    // Between frames, keep the previous values (the timestamp does not change, so Vision
    // ignores the repeated measurement, as with a real Limelight).
    if (lastFrameTime >= 0.0 && now - lastFrameTime < 1.0 / FRAME_RATE_HZ) {
      return;
    }
    lastFrameTime = now;

    Pose2d robotPose = truePoseSupplier.get();
    Pose2d cameraPose = robotPose.transformBy(ROBOT_TO_CAMERA);

    int visibleCount = 0;
    double distanceSum = 0.0;
    double closestDistance = Double.POSITIVE_INFINITY;
    int closestId = -1;
    double closestTxDeg = 0.0;

    for (var tag : layout.getTags()) {
      Pose2d tagPose = tag.pose.toPose2d();
      Translation2d cameraToTag = tagPose.getTranslation().minus(cameraPose.getTranslation());
      double distance = cameraToTag.getNorm();
      if (distance > Constants.Vision.DISTANCIA_MAXIMA_VISAO_METROS || distance < 1e-6) {
        continue;
      }

      // Inside the camera's horizontal field of view?
      Rotation2d bearing = cameraToTag.getAngle().minus(cameraPose.getRotation());
      if (Math.abs(bearing.getDegrees()) > HORIZONTAL_FOV_DEG / 2.0) {
        continue;
      }

      // Is the tag facing the camera? (the tag's X axis points out of its face)
      Rotation2d viewAngle = cameraToTag.unaryMinus().getAngle().minus(tagPose.getRotation());
      if (Math.abs(viewAngle.getDegrees()) > MAX_TAG_VIEW_ANGLE_DEG) {
        continue;
      }

      visibleCount++;
      distanceSum += distance;
      if (distance < closestDistance) {
        closestDistance = distance;
        closestId = tag.ID;
        closestTxDeg = -bearing.getDegrees(); // Limelight tx is positive to the right.
      }
    }

    // ----- General target data -----
    inputs.temAlvo = visibleCount > 0;
    inputs.aprilTagID = closestId;
    inputs.tx = visibleCount > 0 ? closestTxDeg : 0.0;
    inputs.ty = 0.0;

    if (visibleCount == 0) {
      inputs.quantidadeTagsMegaTag1 = 0;
      inputs.quantidadeTagsMegaTag2 = 0;
      inputs.distanciaMediaTagsMegaTag1 = 0.0;
      inputs.distanciaMediaTagsMegaTag2 = 0.0;
      return;
    }

    double averageDistance = distanceSum / visibleCount;
    double ambiguity = visibleCount >= 2 ? 0.0 : SINGLE_TAG_AMBIGUITY;
    double timestamp = now - LATENCY_SECONDS;

    // ----- MegaTag 2: heading comes from the gyro, so only position is noisy -----
    double mt2Noise = MT2_XY_NOISE_PER_METER * averageDistance;
    inputs.poseMegaTag2 =
        new Pose2d(
            robotPose.getX() + random.nextGaussian() * mt2Noise,
            robotPose.getY() + random.nextGaussian() * mt2Noise,
            robotPose.getRotation());
    inputs.timestampMegaTag2 = timestamp;
    inputs.quantidadeTagsMegaTag2 = visibleCount;
    inputs.distanciaMediaTagsMegaTag2 = averageDistance;
    inputs.ambiguidadeMinimaMegaTag2 = ambiguity;

    // ----- MegaTag 1: position and heading both estimated from the tags -----
    double mt1Noise = MT1_XY_NOISE_PER_METER * averageDistance;
    inputs.poseMegaTag1 =
        new Pose2d(
            robotPose.getX() + random.nextGaussian() * mt1Noise,
            robotPose.getY() + random.nextGaussian() * mt1Noise,
            robotPose
                .getRotation()
                .plus(Rotation2d.fromDegrees(random.nextGaussian() * MT1_HEADING_NOISE_DEG)));
    inputs.timestampMegaTag1 = timestamp;
    inputs.quantidadeTagsMegaTag1 = visibleCount;
    inputs.distanciaMediaTagsMegaTag1 = averageDistance;
    inputs.ambiguidadeMinimaMegaTag1 = ambiguity;
  }
}
