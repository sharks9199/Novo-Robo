package frc.robot.commands;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

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
import frc.robot.subsystems.drive.control.HeadingController;

public final class DriveCommands {
    private DriveCommands() {}

    private static final double VELOCIDADE_MINIMA_HEADING_HOLD_MPS = 0.05;

    public static Command joystickDrive(
        Drive drive,
        HeadingController headingController,
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
        SlewRateLimiter limitadorRotacaoManual = new SlewRateLimiter(
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

                double velocidadeX = limitadorX.calculate(
                    movimento.getX() * Constants.Chassi.VELOCIDADE_MAXIMA_MPS
                );
                double velocidadeY = limitadorY.calculate(
                    movimento.getY() * Constants.Chassi.VELOCIDADE_MAXIMA_MPS
                );

                boolean pilotoQuerGirar = Math.abs(comandoRotacao) > 1e-6;
                boolean roboEstaTransladando =
                    Math.hypot(velocidadeX, velocidadeY)
                        > VELOCIDADE_MINIMA_HEADING_HOLD_MPS;

                double velocidadeAngular;

                if (pilotoQuerGirar) {
                    // O motorista sempre tem prioridade sobre o HeadingController.
                    headingController.desativar();

                    comandoRotacao = Math.copySign(
                        comandoRotacao * comandoRotacao,
                        comandoRotacao
                    );

                    double omegaManual =
                        comandoRotacao
                            * Constants.Chassi.VELOCIDADE_ANGULAR_MAXIMA_RAD_S;

                    velocidadeAngular =
                        limitadorRotacaoManual.calculate(omegaManual);

                } else if (roboEstaTransladando) {
                    // Vale para frente, tras, esquerda, direita e diagonal.
                    // Quando começa a transladar sem giro manual, a orientacao
                    // atual vira a referencia de reta.
                    if (!headingController.estaAtivo()) {
                        limitadorRotacaoManual.reset(0.0);
                        headingController.segurarHeadingAtual(
                            drive.getRotation()
                        );
                    }

                    // O PID corrige apenas desvios de yaw enquanto translada.
                    velocidadeAngular = headingController.calcular(
                        drive.getRotation()
                    );

                } else {
                    // Parado e sem giro manual: nao tenta procurar angulo.
                    headingController.desativar();
                    limitadorRotacaoManual.reset(0.0);
                    velocidadeAngular = 0.0;
                }

                headingController.atualizarDashboard(
                    drive.getRotation(),
                    velocidadeAngular
                );

                Logger.recordOutput(
                    "Drive/HeadingController/Ativo",
                    headingController.estaAtivo()
                );
                Logger.recordOutput(
                    "Drive/HeadingController/Transladando",
                    roboEstaTransladando
                );
                Logger.recordOutput(
                    "Drive/HeadingController/GiroManual",
                    pilotoQuerGirar
                );
                Logger.recordOutput(
                    "Drive/HeadingController/AlvoGraus",
                    headingController.getHeadingAlvo().getDegrees()
                );
                Logger.recordOutput(
                    "Drive/HeadingController/AtualGraus",
                    drive.getRotation().getDegrees()
                );
                Logger.recordOutput(
                    "Drive/HeadingController/ErroGraus",
                    Math.toDegrees(headingController.getErroRad())
                );
                Logger.recordOutput(
                    "Drive/HeadingController/OmegaAplicadoRadS",
                    velocidadeAngular
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
        ).finallyDo(interrompido -> {
            headingController.desativar();
            limitadorX.reset(0.0);
            limitadorY.reset(0.0);
            limitadorRotacaoManual.reset(0.0);
            drive.stop();
        });
    }

    private static Translation2d obterMovimentoLinear(double x, double y) {
        x = MathUtil.applyDeadband(x, Constants.Chassi.DEADBAND_JOYSTICK);
        y = MathUtil.applyDeadband(y, Constants.Chassi.DEADBAND_JOYSTICK);

        double magnitude = Math.min(Math.hypot(x, y), 1.0);
        if (magnitude < 1e-9) {
            return new Translation2d();
        }

        Rotation2d direcao = new Rotation2d(Math.atan2(y, x));
        magnitude *= magnitude;

        return new Pose2d(new Translation2d(), direcao)
            .transformBy(new Transform2d(magnitude, 0.0, new Rotation2d()))
            .getTranslation();
    }
}
