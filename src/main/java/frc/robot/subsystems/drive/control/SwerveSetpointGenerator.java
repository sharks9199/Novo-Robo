package frc.robot.subsystems.drive.control;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Gerador de setpoints fisicamente limitados para o modo RETO.
 * Limita taxa de steering e aceleracao de roda antes de enviar os estados aos modulos.
 */
public final class SwerveSetpointGenerator {
    public SwerveModuleState[] generate(
        ChassisSpeeds desiredSpeeds,
        SwerveModuleState[] measuredStates,
        SwerveDriveKinematics kinematics,
        double maxWheelSpeedMps,
        double maxWheelAccelerationMps2,
        double maxSteerRateRadS,
        double minimumTranslationSpeedMps,
        double dtSeconds
    ) {
        SwerveModuleState[] desired = kinematics.toSwerveModuleStates(desiredSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(desired, maxWheelSpeedMps);

        SwerveModuleState[] generated = new SwerveModuleState[desired.length];
        double maxSpeedDelta = maxWheelAccelerationMps2 * dtSeconds;
        double maxAngleDelta = maxSteerRateRadS * dtSeconds;

        for (int i = 0; i < desired.length; i++) {
            SwerveModuleState target = new SwerveModuleState(
                desired[i].speedMetersPerSecond,
                desired[i].angle
            );
            target.optimize(measuredStates[i].angle);

            double angleDelta = MathUtil.angleModulus(
                target.angle.minus(measuredStates[i].angle).getRadians()
            );
            angleDelta = MathUtil.clamp(
                angleDelta,
                -maxAngleDelta,
                maxAngleDelta
            );

            Rotation2d limitedAngle = measuredStates[i].angle.plus(
                Rotation2d.fromRadians(angleDelta)
            );

            double limitedSpeed = MathUtil.clamp(
                target.speedMetersPerSecond,
                measuredStates[i].speedMetersPerSecond - maxSpeedDelta,
                measuredStates[i].speedMetersPerSecond + maxSpeedDelta
            );

            double totalAngleError = MathUtil.angleModulus(
                target.angle.minus(measuredStates[i].angle).getRadians()
            );

            // Cosine compensation reduz skew enquanto o modulo termina de alinhar.
            // A velocidade minima e um PISO DE REDUCAO, nao uma velocidade forcada:
            // se limitedSpeed ja for menor que o piso, nenhuma aceleracao artificial
            // e criada. Se estiver acima, a cosine nao pode derrubar abaixo do piso.
            double absLimitedSpeed = Math.abs(limitedSpeed);
            double cosineScale = Math.max(0.0, Math.cos(totalAngleError));

            if (absLimitedSpeed > 1e-6 && minimumTranslationSpeedMps > 0.0) {
                double minimumScale = Math.min(
                    1.0,
                    minimumTranslationSpeedMps / absLimitedSpeed
                );
                cosineScale = MathUtil.clamp(
                    cosineScale,
                    minimumScale,
                    1.0
                );
            }

            limitedSpeed *= cosineScale;

            generated[i] = new SwerveModuleState(limitedSpeed, limitedAngle);
        }

        return generated;
    }
}
