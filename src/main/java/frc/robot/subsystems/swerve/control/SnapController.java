package frc.robot.subsystems.swerve.control;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

/**
 * Shapes translation joystick input.
 *
 * <p>Applies a radial deadband and an exponential response curve, and "snaps" the input to a
 * single axis when the stick is pushed almost straight forward/backward or sideways. Snapping uses
 * hysteresis (separate enter and exit thresholds) to avoid flickering at the boundary.
 */
public final class SnapController {

  /** Axis the input is currently snapped to. */
  private enum SnapAxis {
    NONE,
    LONGITUDINAL,
    LATERAL
  }

  private SnapAxis snapAxis = SnapAxis.NONE;

  /**
   * Processes raw joystick axes into a shaped translation vector.
   *
   * @param longitudinal forward/backward axis, in [-1, 1]
   * @param lateral left/right axis, in [-1, 1]
   * @return shaped vector with magnitude in [0, 1], or zero inside the deadband
   */
  public Translation2d process(double longitudinal, double lateral) {
    atualizarSnap(longitudinal, lateral);

    double x = longitudinal;
    double y = lateral;

    if (snapAxis == SnapAxis.LONGITUDINAL) {
      y = 0.0;
    } else if (snapAxis == SnapAxis.LATERAL) {
      x = 0.0;
    }

    double rawMagnitude = Math.hypot(x, y);
    double magnitude = Math.min(rawMagnitude, 1.0);
    if (magnitude <= Constants.ControlesDrive.DEADBAND_RADIAL) {
      return Translation2d.kZero;
    }

    double magnitudeSemDeadband =
        MathUtil.applyDeadband(magnitude, Constants.ControlesDrive.DEADBAND_RADIAL);
    double magnitudeModelada =
        Math.pow(magnitudeSemDeadband, Constants.ControlesDrive.EXPO_TRANSLACAO);

    return new Translation2d(x / rawMagnitude, y / rawMagnitude).times(magnitudeModelada);
  }

  /** Updates the snap state machine (with hysteresis) from the latest raw input. */
  private void atualizarSnap(double longitudinal, double lateral) {
    double absLongitudinal = Math.abs(longitudinal);
    double absLateral = Math.abs(lateral);

    // Stick too close to center: release any snap.
    if (Math.max(absLongitudinal, absLateral)
        < Constants.ControlesDrive.EIXO_PRINCIPAL_MINIMO_SNAP) {
      snapAxis = SnapAxis.NONE;
      return;
    }

    // Already snapped: stay snapped until the secondary axis exceeds the exit threshold.
    if (snapAxis == SnapAxis.LONGITUDINAL) {
      if (absLateral > Constants.ControlesDrive.EIXO_SECUNDARIO_SAIR_SNAP) {
        snapAxis = SnapAxis.NONE;
      }
      return;
    }

    if (snapAxis == SnapAxis.LATERAL) {
      if (absLongitudinal > Constants.ControlesDrive.EIXO_SECUNDARIO_SAIR_SNAP) {
        snapAxis = SnapAxis.NONE;
      }
      return;
    }

    // Not snapped: engage when the secondary axis is small enough.
    if (absLongitudinal >= absLateral
        && absLateral <= Constants.ControlesDrive.EIXO_SECUNDARIO_ENTRAR_SNAP) {
      snapAxis = SnapAxis.LONGITUDINAL;
    } else if (absLateral > absLongitudinal
        && absLongitudinal <= Constants.ControlesDrive.EIXO_SECUNDARIO_ENTRAR_SNAP) {
      snapAxis = SnapAxis.LATERAL;
    }
  }

  /** Returns whether the input is currently snapped to the forward/backward axis. */
  public boolean isSnappingLongitudinal() {
    return snapAxis == SnapAxis.LONGITUDINAL;
  }

  /** Returns whether the input is currently snapped to the left/right axis. */
  public boolean isSnappingLateral() {
    return snapAxis == SnapAxis.LATERAL;
  }

  /** Clears the snap state. */
  public void reset() {
    snapAxis = SnapAxis.NONE;
  }
}
