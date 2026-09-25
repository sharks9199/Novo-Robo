package frc.robot.subsystems.drive.control;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Limitador vetorial de aceleracao e jerk. Diferente de limitar X/Y isoladamente,
 * preserva o limite total em qualquer direcao do swerve.
 */
public final class JerkLimiter2d {
    private final double maxAcceleration;
    private final double maxJerk;
    private final double periodSeconds;

    private Translation2d velocity = Translation2d.kZero;
    private Translation2d acceleration = Translation2d.kZero;

    public JerkLimiter2d(double maxAcceleration, double maxJerk, double periodSeconds) {
        this.maxAcceleration = Math.abs(maxAcceleration);
        this.maxJerk = Math.abs(maxJerk);
        this.periodSeconds = periodSeconds;
    }

    public Translation2d calculate(Translation2d targetVelocity) {
        Translation2d velocityError = targetVelocity.minus(velocity);
        Translation2d desiredAcceleration = velocityError.div(periodSeconds);
        desiredAcceleration = clampMagnitude(desiredAcceleration, maxAcceleration);

        Translation2d accelerationDelta = desiredAcceleration.minus(acceleration);
        accelerationDelta = clampMagnitude(
            accelerationDelta,
            maxJerk * periodSeconds
        );

        acceleration = clampMagnitude(
            acceleration.plus(accelerationDelta),
            maxAcceleration
        );

        Translation2d nextVelocity = velocity.plus(
            acceleration.times(periodSeconds)
        );

        Translation2d velocityStep = nextVelocity.minus(velocity);
        double stepAlongError =
            velocityStep.getX() * velocityError.getX()
            + velocityStep.getY() * velocityError.getY();
        if (
            stepAlongError > 0.0
            && velocityStep.getNorm() >= velocityError.getNorm()
        ) {
            nextVelocity = targetVelocity;
            acceleration = Translation2d.kZero;
        }

        velocity = nextVelocity;
        return velocity;
    }

    public void reset(Translation2d velocity) {
        this.velocity = velocity;
        acceleration = Translation2d.kZero;
    }

    public Translation2d getVelocity() {
        return velocity;
    }

    public Translation2d getAcceleration() {
        return acceleration;
    }

    private static Translation2d clampMagnitude(
        Translation2d vector,
        double maxMagnitude
    ) {
        double norm = vector.getNorm();
        if (norm <= maxMagnitude || norm < 1e-9) {
            return vector;
        }
        return vector.times(maxMagnitude / norm);
    }
}
