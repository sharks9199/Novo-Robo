package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.Drive;
import org.littletonrobotics.junction.Logger;

/** Reads AprilTag poses and fuses validated vision measurements into Drive odometry. */
public class Vision extends SubsystemBase {
  /** Limelight pipeline index configured for AprilTag detection. */
  private static final int PIPELINE_APRILTAG = 0;

  /** Minimum number of tags required to accept a measurement. */
  private static final int TAGS_MINIMAS = 1;

  private final Drive drive;
  private final VisionIO io;
  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

  /** Timestamp of the last accepted measurement; prevents fusing the same frame twice. */
  private double ultimoTimestampAceito = -1.0;

  /**
   * Creates the vision subsystem and selects the AprilTag pipeline.
   *
   * @param drive the drive subsystem that receives vision measurements
   * @param io camera hardware abstraction
   */
  public Vision(Drive drive, VisionIO io) {
    this.drive = drive;
    this.io = io;
    io.definirPipeline(PIPELINE_APRILTAG);
  }

  @Override
  public void periodic() {
    io.definirOrientacao(
        drive.getRawGyroRotation().getDegrees(), Math.toDegrees(drive.getYawVelocityRadPerSec()));
    io.updateInputs(inputs);
    Logger.processInputs("Vision/Limelight", inputs);

    Logger.recordOutput("Vision/AprilTag/Visible", inputs.temAlvo);
    Logger.recordOutput("Vision/AprilTag/ID", inputs.aprilTagID);
    Logger.recordOutput("Vision/MegaTag1/TagCount", inputs.quantidadeTagsMegaTag1);
    Logger.recordOutput("Vision/MegaTag2/TagCount", inputs.quantidadeTagsMegaTag2);
    Logger.recordOutput("Vision/MegaTag2/DistanceMeters", inputs.distanciaMediaTagsMegaTag2);

    // Prefer MegaTag2; it uses the Pigeon orientation supplied above and does not
    // estimate heading from the tag geometry. Never fuse the same camera frame twice.
    if (medicaoValida(
        inputs.poseMegaTag2,
        inputs.timestampMegaTag2,
        inputs.quantidadeTagsMegaTag2,
        inputs.distanciaMediaTagsMegaTag2)) {
      int tags = inputs.quantidadeTagsMegaTag2;
      double sigmaXY =
          tags >= 2
              ? Constants.Vision.MT2_DESVIO_XY_MULTIPLAS_TAGS_METROS
              : Constants.Vision.MT2_DESVIO_XY_TAG_UNICA_METROS;
      drive.adicionarMedicaoVisao(
          inputs.poseMegaTag2,
          inputs.timestampMegaTag2,
          sigmaXY,
          sigmaXY,
          Constants.Vision.MT2_DESVIO_THETA_RAD);
      ultimoTimestampAceito = inputs.timestampMegaTag2;
      Logger.recordOutput("Vision/Accepted", true);
      Logger.recordOutput("Vision/AcceptedSource", "MegaTag2");
      return;
    }

    // Fall back to MegaTag1 only if MT2 is unavailable/invalid. Use heading only
    // with multiple tags; a single tag's heading is too ambiguous for reliable fusion.
    if (medicaoValida(
        inputs.poseMegaTag1,
        inputs.timestampMegaTag1,
        inputs.quantidadeTagsMegaTag1,
        inputs.distanciaMediaTagsMegaTag1)) {
      int tags = inputs.quantidadeTagsMegaTag1;
      double sigmaXY =
          tags >= 2
              ? Constants.Vision.MT1_DESVIO_XY_MULTIPLAS_TAGS_METROS
              : Constants.Vision.MT1_DESVIO_XY_TAG_UNICA_METROS;
      double sigmaTheta =
          tags >= 2
              ? Constants.Vision.MT1_DESVIO_THETA_MULTIPLAS_TAGS_RAD
              : Constants.Vision.MT1_DESVIO_THETA_TAG_UNICA_RAD;
      drive.adicionarMedicaoVisao(
          inputs.poseMegaTag1, inputs.timestampMegaTag1, sigmaXY, sigmaXY, sigmaTheta);
      ultimoTimestampAceito = inputs.timestampMegaTag1;
      Logger.recordOutput("Vision/Accepted", true);
      Logger.recordOutput("Vision/AcceptedSource", "MegaTag1");
    } else {
      Logger.recordOutput("Vision/Accepted", false);
      Logger.recordOutput("Vision/AcceptedSource", "None");
    }
  }

  /**
   * Checks whether a vision measurement is safe to fuse into odometry.
   *
   * @param pose measured robot pose
   * @param timestamp capture timestamp, in seconds
   * @param tags number of tags used
   * @param distancia average distance to the tags, in meters
   * @return {@code true} if the measurement should be accepted
   */
  private boolean medicaoValida(Pose2d pose, double timestamp, int tags, double distancia) {
    if (tags < TAGS_MINIMAS
        || !Double.isFinite(timestamp)
        || timestamp <= 0.0
        || timestamp <= ultimoTimestampAceito
        || !Double.isFinite(pose.getX())
        || !Double.isFinite(pose.getY())
        || !Double.isFinite(distancia)
        || distancia <= 0.0
        || distancia > Constants.Vision.DISTANCIA_MAXIMA_VISAO_METROS) {
      return false;
    }

    // Reject measurements while rotating too fast (motion blur / latency error).
    if (Math.abs(drive.getYawVelocityRadPerSec())
        > Constants.Vision.VELOCIDADE_ANGULAR_MAXIMA_PARA_VISAO_RAD_S) {
      return false;
    }

    // Reject estimates far from the current odometry. The tolerance scales with range
    // and tag count; this guards against wrong field setup or transient bad solves.
    double erro = pose.getTranslation().getDistance(drive.getPose().getTranslation());
    double tolerancia = Math.max(1.0, distancia * (tags >= 2 ? 1.5 : 1.0));
    if (erro > tolerancia) {
      return false;
    }

    // A single tag with high ambiguity is unreliable.
    if (tags == 1
        && inputs.ambiguidadeMinimaMegaTag2 > Constants.Vision.AMBIGUIDADE_MAXIMA_TAG_UNICA_HEADING
        && inputs.ambiguidadeMinimaMegaTag2 > 0.0) {
      return false;
    }

    return !DriverStation.isEStopped();
  }
}