package frc.robot.commands.features;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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
        DoubleSupplier rotacao
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

    private static Translation2d obterMovimentoLinear(double x, double y) {
        x = MathUtil.applyDeadband(x, Constants.Chassi.DEADBAND_JOYSTICK);
        y = MathUtil.applyDeadband(y, Constants.Chassi.DEADBAND_JOYSTICK);
        double magnitude = Math.min(Math.hypot(x, y), 1.0);
        if (magnitude < 1e-9) return new Translation2d();
        Rotation2d direcao = new Rotation2d(Math.atan2(y, x));
        magnitude *= magnitude;
        return new Pose2d(new Translation2d(), direcao).transformBy(new Transform2d(magnitude, 0.0, new Rotation2d())).getTranslation();
    }
}
