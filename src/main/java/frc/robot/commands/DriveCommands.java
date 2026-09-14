package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;

public final class DriveCommands {
    private DriveCommands() {}

    public static Command joystickDrive(
        Drive drive,
        DoubleSupplier frenteTras,
        DoubleSupplier lateral,
        DoubleSupplier rotacao,
        BooleanSupplier limitarHeading
    ) {
        SlewRateLimiter limitadorX = new SlewRateLimiter(
            Constants.Chassi.ACELERACAO_MAXIMA_MPS2
        );
        SlewRateLimiter limitadorY = new SlewRateLimiter(
            Constants.Chassi.ACELERACAO_MAXIMA_MPS2
        );
        SlewRateLimiter limitadorRotacao = new SlewRateLimiter(
            Constants.Chassi.ACELERACAO_ANGULAR_MAXIMA_RAD_S2
        );

        return Commands.run(
            () -> {
                Translation2d movimento = obterMovimentoLinear(
                    frenteTras.getAsDouble(),
                    lateral.getAsDouble()
                );

                double comandoRotacao = MathUtil.applyDeadband(
                    rotacao.getAsDouble(),
                    Constants.Chassi.DEADBAND_JOYSTICK
                );
                comandoRotacao = Math.copySign(
                    comandoRotacao * comandoRotacao,
                    comandoRotacao
                );

                double velocidadeX = limitadorX.calculate(
                    movimento.getX() * Constants.Chassi.VELOCIDADE_MAXIMA_MPS
                );
                double velocidadeY = limitadorY.calculate(
                    movimento.getY() * Constants.Chassi.VELOCIDADE_MAXIMA_MPS
                );
                double velocidadeAngular = limitadorRotacao.calculate(
                    comandoRotacao
                        * Constants.Chassi.VELOCIDADE_ANGULAR_MAXIMA_RAD_S
                );

                if (limitarHeading.getAsBoolean()) {
                    double velocidadeAngularLimitada = limitarRotacaoPorHeading(
                        velocidadeAngular,
                        drive.getPose().getRotation()
                    );

                    if (
                        Math.abs(
                            velocidadeAngularLimitada - velocidadeAngular
                        ) > 1e-9
                    ) {
                        // Mantém o estado interno do filtro coerente com a
                        // velocidade realmente enviada ao drivetrain. Assim,
                        // ao inverter o analógico no limite, o robô responde
                        // imediatamente para o sentido permitido.
                        limitadorRotacao.reset(velocidadeAngularLimitada);
                    }

                    velocidadeAngular = velocidadeAngularLimitada;
                }

                drive.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        velocidadeX,
                        velocidadeY,
                        velocidadeAngular,
                        drive.getRotation()
                    )
                );
            },
            drive
        ).finallyDo(drive::stop);
    }

    private static double limitarRotacaoPorHeading(
        double velocidadeAngularRadS,
        Rotation2d headingDoCampo
    ) {
        if (Math.abs(velocidadeAngularRadS) < 1e-9) {
            return 0.0;
        }

        double headingGraus = MathUtil.inputModulus(
            headingDoCampo.getDegrees(),
            -180.0,
            180.0
        );

        double distanciaAteLimiteGraus = velocidadeAngularRadS > 0.0
            ? Constants.Chassi.HEADING_MAXIMO_GRAUS - headingGraus
            : headingGraus - Constants.Chassi.HEADING_MINIMO_GRAUS;

        if (
            distanciaAteLimiteGraus
                <= Constants.Chassi.TOLERANCIA_LIMITE_HEADING_GRAUS
        ) {
            return 0.0;
        }

        double distanciaAteLimiteRad = Math.toRadians(
            distanciaAteLimiteGraus
        );

        double velocidadeMaximaParaPararNoLimite = Math.sqrt(
            2.0
                * Constants.Chassi.ACELERACAO_ANGULAR_MAXIMA_RAD_S2
                * distanciaAteLimiteRad
        );

        double magnitudeLimitada = Math.min(
            Math.abs(velocidadeAngularRadS),
            velocidadeMaximaParaPararNoLimite
        );

        return Math.copySign(magnitudeLimitada, velocidadeAngularRadS);
    }

    private static Translation2d obterMovimentoLinear(double x, double y) {
        x = MathUtil.applyDeadband(x, Constants.Chassi.DEADBAND_JOYSTICK);
        y = MathUtil.applyDeadband(y, Constants.Chassi.DEADBAND_JOYSTICK);

        double magnitude = Math.min(Math.hypot(x, y), 1.0);
        if (magnitude < 1e-9) {
            return Translation2d.kZero;
        }

        Rotation2d direcao = new Rotation2d(Math.atan2(y, x));
        magnitude *= magnitude;

        return new Translation2d(magnitude, direcao);
    }
}
