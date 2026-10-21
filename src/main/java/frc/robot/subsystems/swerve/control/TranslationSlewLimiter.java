package frc.robot.subsystems.swerve.control;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Limits the rate of change (acceleration) of a 2D velocity vector.
 *
 * <p>Unlike limiting each axis independently, this limits the magnitude of the change, so the
 * direction of motion is preserved while ramping toward the target.
 */
public final class TranslationSlewLimiter {
  private final double maxAccelerationMps2;
  private final double periodSeconds;
  private Translation2d current = Translation2d.kZero;

  /**
   * Creates a limiter.
   *
   * @param maxAccelerationMps2 maximum acceleration, in m/s^2 (sign is ignored)
   * @param periodSeconds time between calls to {@link #calculate}, in seconds
   */
  public TranslationSlewLimiter(double maxAccelerationMps2, double periodSeconds) {
    this.maxAccelerationMps2 = Math.abs(maxAccelerationMps2);
    this.periodSeconds = periodSeconds;
  }

  /**
   * Moves the output toward the target without exceeding the acceleration limit.
   *
   * @param target desired velocity vector, in m/s
   * @return the rate-limited velocity vector, in m/s
   */
  public Translation2d calculate(Translation2d target) {
    Translation2d delta = target.minus(current);
    double distance = delta.getNorm();
    double maxStep = maxAccelerationMps2 * periodSeconds;

    if (distance <= maxStep) {
      current = target;
    } else {
      current = current.plus(delta.times(maxStep / distance));
    }
    return current;
  }

  /**
   * Resets the limiter's output.
   *
   * @param value the new current velocity vector, in m/s
   */
  public void reset(Translation2d value) {
    current = value;
  }
}
