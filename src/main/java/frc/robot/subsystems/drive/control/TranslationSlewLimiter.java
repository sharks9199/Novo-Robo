package frc.robot.subsystems.drive.control;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Limita a taxa de variacao do VETOR de velocidade (vx, vy) em m/s².
 *
 * Diferente de dois SlewRateLimiter (um por eixo), o limite vale para a magnitude
 * da variacao em qualquer direcao, entao o tempo para atingir a velocidade maxima
 * e o mesmo indo para frente, para o lado ou na diagonal.
 */
public final class TranslationSlewLimiter {
    private final double maxAccelerationMps2;
    private final double periodSeconds;
    private Translation2d current = Translation2d.kZero;

    public TranslationSlewLimiter(double maxAccelerationMps2, double periodSeconds) {
        this.maxAccelerationMps2 = Math.abs(maxAccelerationMps2);
        this.periodSeconds = periodSeconds;
    }

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

    public void reset(Translation2d value) {
        current = value;
    }
}
