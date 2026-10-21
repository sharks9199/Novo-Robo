package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;

/**
 * Hardware-specific calibration offsets.
 *
 * <p>Values here are measured on the physical robot and must be re-calibrated whenever a module,
 * encoder or sensor is removed, replaced or remounted.
 */
public final class OffsetsConstants {

  /** Driver-station orientation offset. */
  public static final class Driver {
    /** Field-relative "forward" direction as seen by the driver, in degrees. */
    public static final double FRENTE_GRAUS = 0.0;

    private Driver() {}
  }

  /** Absolute encoder (CANcoder) magnet offsets for each swerve module. */
  public static final class Swerve {
    /** Front-left module encoder offset. */
    public static final Angle FRENTE_ESQUERDA = Rotations.of(-0.13671875);

    /** Front-right module encoder offset. */
    public static final Angle FRENTE_DIREITA = Rotations.of(0.29541015625);

    /** Back-left module encoder offset. */
    public static final Angle TRAS_ESQUERDA = Rotations.of(0.199462890625);

    /** Back-right module encoder offset. */
    public static final Angle TRAS_DIREITA = Rotations.of(-0.366455078125);

    private Swerve() {}
  }

  /** Limelight aiming offsets. */
  public static final class Limelight {
    /** Target horizontal offset (tx) considered "centered", in degrees. */
    public static final double TX_ALVO_GRAUS = 0.0;

    private Limelight() {}
  }

  /** Pigeon 2 mounting orientation relative to the robot frame. */
  public static final class Pigeon {
    public static final Angle YAW = Degrees.of(-90.0);
    public static final Angle PITCH = Degrees.of(0.0);
    public static final Angle ROLL = Degrees.of(0.0);

    private Pigeon() {}
  }

  private OffsetsConstants() {}
}
