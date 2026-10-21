package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

/** Hardware abstraction for the AprilTag camera (Limelight). */
public interface VisionIO {

  /** Vision inputs logged by AdvantageKit. */
  @AutoLog
  class VisionIOInputs {
    // ----- General target data -----
    public boolean temAlvo = false;
    public int aprilTagID = -1;
    public double tx = 0.0;
    public double ty = 0.0;

    // ----- MegaTag 1 (vision-only pose) -----
    public int quantidadeTagsMegaTag1 = 0;
    public double distanciaMediaTagsMegaTag1 = 0.0;
    public double ambiguidadeMinimaMegaTag1 = 1.0;
    public Pose2d poseMegaTag1 = new Pose2d();
    public double timestampMegaTag1 = 0.0;

    // ----- MegaTag 2 (gyro-assisted pose) -----
    public int quantidadeTagsMegaTag2 = 0;
    public double distanciaMediaTagsMegaTag2 = 0.0;
    public double ambiguidadeMinimaMegaTag2 = 1.0;
    public Pose2d poseMegaTag2 = new Pose2d();
    public double timestampMegaTag2 = 0.0;
  }

  /** Refreshes the given inputs with the latest camera data. */
  default void updateInputs(VisionIOInputs inputs) {}

  /** Selects the camera pipeline. */
  default void definirPipeline(int pipeline) {}

  /**
   * Sends the robot orientation to the camera (required by MegaTag 2).
   *
   * @param yawGraus robot yaw, in degrees
   * @param yawRateGrausPorSegundo robot yaw rate, in degrees per second
   */
  default void definirOrientacao(double yawGraus, double yawRateGrausPorSegundo) {}
}
