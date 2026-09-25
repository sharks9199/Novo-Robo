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
        double minCosineScale,
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

            // Cosine compensation reduz skew enquanto o modulo termina de alinhar,
            // mas nao deixa mais a roda praticamente parar. O piso e proporcional
            // ao comando atual, portanto nao cria uma velocidade minima absoluta.
            double cosineScale = MathUtil.clamp(
                Math.cos(totalAngleError),
                minCosineScale,
                1.0
            );
            limitedSpeed *= cosineScale;

            generated[i] = new SwerveModuleState(limitedSpeed, limitedAngle);
        }

        return generated;
    }
}
