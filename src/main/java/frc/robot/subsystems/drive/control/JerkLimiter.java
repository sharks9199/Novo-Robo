package frc.robot.subsystems.drive.control;

import edu.wpi.first.math.MathUtil;

/**
 * Limita velocidade, aceleracao e jerk de um comando escalar.
 * Mantem o estado entre ciclos de 20 ms.
 */
public final class JerkLimiter {
    private final double maxAcceleration;
    private final double maxJerk;
    private final double periodSeconds;

    private double velocity;
    private double acceleration;

    public JerkLimiter(double maxAcceleration, double maxJerk, double periodSeconds) {
        this.maxAcceleration = Math.abs(maxAcceleration);
        this.maxJerk = Math.abs(maxJerk);
        this.periodSeconds = periodSeconds;
    }

    public double calculate(double targetVelocity) {
        double error = targetVelocity - velocity;
        double desiredAcceleration = MathUtil.clamp(
            error / periodSeconds,
            -maxAcceleration,
            maxAcceleration
        );

        double maxAccelerationChange = maxJerk * periodSeconds;
        acceleration += MathUtil.clamp(
            desiredAcceleration - acceleration,
            -maxAccelerationChange,
            maxAccelerationChange
        );
        acceleration = MathUtil.clamp(
            acceleration,
            -maxAcceleration,
            maxAcceleration
        );

        double nextVelocity = velocity + acceleration * periodSeconds;

        double velocityStep = nextVelocity - velocity;
        if (
            Math.signum(velocityStep) == Math.signum(error)
            && Math.abs(velocityStep) >= Math.abs(error)
        ) {
            nextVelocity = targetVelocity;
            acceleration = 0.0;
        }

        velocity = nextVelocity;
        return velocity;
    }

    public void reset(double velocity) {
        this.velocity = velocity;
        acceleration = 0.0;
    }

    public double getVelocity() {
        return velocity;
    }

    public double getAcceleration() {
        return acceleration;
    }
}
