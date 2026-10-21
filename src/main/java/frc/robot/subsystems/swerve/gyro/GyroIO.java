package frc.robot.subsystems.swerve.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Hardware abstraction for the drivetrain gyroscope. */
public interface GyroIO {

  /** Gyro inputs logged by AdvantageKit. */
  @AutoLog
  public static class GyroIOInputs {
    /** Whether the gyro is connected and reporting valid data. */
    public boolean connected = false;

    /** Current yaw angle. */
    public Rotation2d yawPosition = new Rotation2d();

    /** Current yaw rate, in rad/s. */
    public double yawVelocityRadPerSec = 0.0;

    /** Timestamps of the high-frequency odometry yaw samples, in seconds. */
    public double[] odometryYawTimestamps = new double[] {};

    /** High-frequency odometry yaw samples. */
    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
  }

  /**
   * Refreshes the given inputs with the latest gyro data.
   *
   * @param inputs the inputs object to populate
   */
  public default void updateInputs(GyroIOInputs inputs) {}
}
