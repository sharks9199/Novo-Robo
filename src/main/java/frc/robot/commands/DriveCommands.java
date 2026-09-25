package frc.robot.commands;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.control.DirectionalInputProcessor;
import frc.robot.subsystems.drive.control.HeadingController;
import frc.robot.subsystems.drive.control.JerkLimiter;
import frc.robot.subsystems.drive.control.JerkLimiter2d;
import frc.robot.subsystems.drive.control.ModoRetoController;

public final class DriveCommands {
    private DriveCommands() {}

    private static final class EstadoTeleop {
        boolean modoRetoAnterior = false;
        boolean aguardandoFimGiroManual = false;
        double ultimoVxField = 0.0;
        double ultimoVyField = 0.0;
        double ultimoOmega = 0.0;
    }

    public static Command joystickDrive(
        Drive drive,
        HeadingController headingController,
        ModoRetoController modoReto,
        DoubleSupplier frenteTras,
        DoubleSupplier lateral,
        DoubleSupplier rotacao
    ) {
        DirectionalInputProcessor inputProcessor = new DirectionalInputProcessor();

        SlewRateLimiter normalX = new SlewRateLimiter(
            Constants.Chassi.ACELERACAO_MAXIMA_MPS2
        );
        SlewRateLimiter normalY = new SlewRateLimiter(
            Constants.Chassi.ACELERACAO_MAXIMA_MPS2
        );
        SlewRateLimiter normalOmega = new SlewRateLimiter(
            Constants.Chassi.ACELERACAO_ANGULAR_MAXIMA_RAD_S2
        );

        JerkLimiter2d retoTranslation = new JerkLimiter2d(
            Constants.ModoReto.ACELERACAO_TRANSLACAO_MAXIMA_MPS2,
            Constants.ModoReto.JERK_TRANSLACAO_MAXIMO_MPS3,
            Constants.Chassi.PERIODO_CONTROLE_SEGUNDOS
        );
        JerkLimiter retoOmega = new JerkLimiter(
            Constants.Chassi.ACELERACAO_ANGULAR_MAXIMA_RAD_S2,
            Constants.ModoReto.JERK_ANGULAR_MAXIMO_RAD_S3,
            Constants.Chassi.PERIODO_CONTROLE_SEGUNDOS
        );

        EstadoTeleop estado = new EstadoTeleop();

        return Commands.run(
            () -> {
                double rawLongitudinal = frenteTras.getAsDouble();
                double rawLateral = lateral.getAsDouble();
                double rawRotation = rotacao.getAsDouble();

                boolean modoLigado = modoReto.estaLigado();
                double expoTranslacao = modoLigado
                    ? Constants.ModoReto.EXPO_TRANSLACAO
                    : Constants.ControlesDrive.EXPO_TRANSLACAO;

                Translation2d normalizedTranslation = inputProcessor.process(
                    rawLongitudinal,
                    rawLateral,
                    expoTranslacao
                );

                double rotationInput = MathUtil.applyDeadband(
                    rawRotation,
                    Constants.Chassi.DEADBAND_JOYSTICK
                );
                boolean manualTurn = Math.abs(rotationInput) > 1e-6;
                rotationInput = Math.copySign(
                    rotationInput * rotationInput,
                    rotationInput
                );

                // O modo normal preserva a velocidade maxima do chassi.
                // O Modo RETO possui limite proprio de 3,0 m/s.
                double velocidadeMaximaTranslacao = modoLigado
                    ? Constants.ModoReto.VELOCIDADE_TRANSLACAO_MAXIMA_MPS
                    : Constants.Chassi.VELOCIDADE_MAXIMA_MPS;

                Translation2d targetFieldVelocity = normalizedTranslation.times(
                    velocidadeMaximaTranslacao
                );
                double targetManualOmega = rotationInput
                    * Constants.Chassi.VELOCIDADE_ANGULAR_MAXIMA_RAD_S;
                if (modoLigado && !estado.modoRetoAnterior) {
                    // Entra no RETO sem parar o robo. Inicializa os limitadores com
                    // a velocidade realmente medida no frame atual para evitar
                    // degrau/travamento ao alternar o modo em movimento.
                    ChassisSpeeds measuredRobot = drive.getChassisSpeeds();
                    ChassisSpeeds measuredField = ChassisSpeeds.fromRobotRelativeSpeeds(
                        measuredRobot,
                        drive.getRotation()
                    );
                    retoTranslation.reset(
                        new Translation2d(
                            measuredField.vxMetersPerSecond,
                            measuredField.vyMetersPerSecond
                        )
                    );
                    retoOmega.reset(measuredRobot.omegaRadiansPerSecond);
                    headingController.definirHeading(
                        Rotation2d.kZero,
                        drive.getRotation(),
                        drive.getYawVelocityRadPerSec()
                    );
                    estado.aguardandoFimGiroManual = false;
                } else if (!modoLigado && estado.modoRetoAnterior) {
                    headingController.desativar();
                    ChassisSpeeds measuredRobot = drive.getChassisSpeeds();
                    ChassisSpeeds measuredField = ChassisSpeeds.fromRobotRelativeSpeeds(
                        measuredRobot,
                        drive.getRotation()
                    );
                    normalX.reset(measuredField.vxMetersPerSecond);
                    normalY.reset(measuredField.vyMetersPerSecond);
                    normalOmega.reset(measuredRobot.omegaRadiansPerSecond);
                    estado.aguardandoFimGiroManual = false;
                }
                estado.modoRetoAnterior = modoLigado;

                if (!modoLigado) {
                    executarModoNormal(
                        drive,
                        headingController,
                        normalX,
                        normalY,
                        normalOmega,
                        targetFieldVelocity,
                        targetManualOmega,
                        estado
                    );
                } else {
                    executarModoReto(
                        drive,
                        headingController,
                        modoReto,
                        retoTranslation,
                        retoOmega,
                        targetFieldVelocity,
                        targetManualOmega,
                        manualTurn,
                        estado
                    );
                }

                publicarTelemetria(
                    drive,
                    headingController,
                    modoReto,
                    inputProcessor,
                    rawLongitudinal,
                    rawLateral,
                    rawRotation,
                    normalizedTranslation,
                    manualTurn,
                    estado
                );
            },
            drive
        ).finallyDo(interrupted -> {
            headingController.desativar();
            inputProcessor.reset();
            normalX.reset(0.0);
            normalY.reset(0.0);
            normalOmega.reset(0.0);
            retoTranslation.reset(Translation2d.kZero);
            retoOmega.reset(0.0);
            drive.stop();
        });
    }

    private static void executarModoNormal(
        Drive drive,
        HeadingController headingController,
        SlewRateLimiter xLimiter,
        SlewRateLimiter yLimiter,
        SlewRateLimiter omegaLimiter,
        Translation2d targetFieldVelocity,
        double targetOmega,
        EstadoTeleop estado
    ) {
        headingController.desativar();

        double vx = xLimiter.calculate(targetFieldVelocity.getX());
        double vy = yLimiter.calculate(targetFieldVelocity.getY());
        double omega = omegaLimiter.calculate(targetOmega);

        drive.runVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                vx,
                vy,
                omega,
                drive.getRotation()
            )
        );

        estado.ultimoVxField = vx;
        estado.ultimoVyField = vy;
        estado.ultimoOmega = omega;
    }

    private static void executarModoReto(
        Drive drive,
        HeadingController headingController,
        ModoRetoController modoReto,
        JerkLimiter2d translationLimiter,
        JerkLimiter omegaLimiter,
        Translation2d targetFieldVelocity,
        double targetManualOmega,
        boolean manualTurn,
        EstadoTeleop estado
    ) {
        Translation2d velocity = translationLimiter.calculate(targetFieldVelocity);
        double targetOmega;

        if (manualTurn) {
            headingController.desativar();
            estado.aguardandoFimGiroManual = true;
            targetOmega = targetManualOmega;
        } else if (estado.aguardandoFimGiroManual) {
            targetOmega = 0.0;
        } else {
            Rotation2d headingRelativo = drive.getRotation();

            if (
                modoReto.deveSnaparZero(headingRelativo)
                && (
                    !headingController.estaAtivo()
                    || Math.abs(headingController.getHeadingAlvo().getRadians()) > 1e-6
                )
            ) {
                headingController.definirHeading(
                    Rotation2d.kZero,
                    headingRelativo,
                    drive.getYawVelocityRadPerSec()
                );
            } else if (!headingController.estaAtivo()) {
                headingController.segurarHeadingAtual(
                    headingRelativo,
                    drive.getYawVelocityRadPerSec()
                );
            }

            targetOmega = headingController.calcular(headingRelativo);
        }

        double omega = omegaLimiter.calculate(targetOmega);

        if (
            !manualTurn
            && estado.aguardandoFimGiroManual
            && Math.abs(drive.getYawVelocityRadPerSec())
                <= Constants.Heading.TOLERANCIA_VELOCIDADE_RAD_S
            && Math.abs(omegaLimiter.getVelocity())
                <= Constants.Heading.TOLERANCIA_VELOCIDADE_RAD_S
        ) {
            estado.aguardandoFimGiroManual = false;
            omegaLimiter.reset(0.0);

            Rotation2d headingRelativo = drive.getRotation();
            Rotation2d alvo = modoReto.deveSnaparZero(headingRelativo)
                ? Rotation2d.kZero
                : headingRelativo;

            headingController.definirHeading(
                alvo,
                headingRelativo,
                drive.getYawVelocityRadPerSec()
            );
            omega = 0.0;
        }

        drive.runVelocityAdvanced(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                velocity.getX(),
                velocity.getY(),
                omega,
                drive.getRotation()
            )
        );

        estado.ultimoVxField = velocity.getX();
        estado.ultimoVyField = velocity.getY();
        estado.ultimoOmega = omega;
    }

    private static void publicarTelemetria(
        Drive drive,
        HeadingController headingController,
        ModoRetoController modoReto,
        DirectionalInputProcessor inputProcessor,
        double rawLongitudinal,
        double rawLateral,
        double rawRotation,
        Translation2d processedTranslation,
        boolean manualTurn,
        EstadoTeleop estado
    ) {
        headingController.atualizarDashboard(
            drive.getRotation(),
            estado.ultimoOmega
        );
        modoReto.publicarDashboard();

        SmartDashboard.putBoolean(
            "DriveInput/SnapLongitudinal",
            inputProcessor.isSnappingLongitudinal()
        );
        SmartDashboard.putBoolean(
            "DriveInput/SnapLateral",
            inputProcessor.isSnappingLateral()
        );
        SmartDashboard.putNumber("DriveInput/RawLongitudinal", rawLongitudinal);
        SmartDashboard.putNumber("DriveInput/RawLateral", rawLateral);
        SmartDashboard.putNumber("DriveInput/RawRotation", rawRotation);
        SmartDashboard.putNumber(
            "DriveInput/ProcessedLongitudinal",
            processedTranslation.getX()
        );
        SmartDashboard.putNumber(
            "DriveInput/ProcessedLateral",
            processedTranslation.getY()
        );

        SmartDashboard.putBoolean("ModoReto/ManualTurn", manualTurn);
        SmartDashboard.putBoolean(
            "ModoReto/AguardandoFimGiro",
            estado.aguardandoFimGiroManual
        );
        SmartDashboard.putNumber(
            "ModoReto/HeadingRelativoDeg",
            drive.getRotation().getDegrees()
        );
        SmartDashboard.putBoolean(
            "ModoReto/DentroJanela20Deg",
            modoReto.deveSnaparZero(drive.getRotation())
        );
        SmartDashboard.putNumber(
            "ModoReto/MaxModuleAlignmentErrorDeg",
            drive.getMaxModuleAlignmentErrorDeg()
        );
        SmartDashboard.putNumber(
            "ModoReto/VelocidadeMaximaMps",
            Constants.ModoReto.VELOCIDADE_TRANSLACAO_MAXIMA_MPS
        );
        SmartDashboard.putNumber(
            "ModoReto/VelocidadeMinimaMps",
            Constants.ModoReto.VELOCIDADE_TRANSLACAO_MINIMA_MPS
        );
        SmartDashboard.putNumber(
            "ModoReto/VelocidadeComandadaMps",
            Math.hypot(estado.ultimoVxField, estado.ultimoVyField)
        );
        SmartDashboard.putNumber(
            "ModoReto/VelocidadeMedidaMps",
            drive.getTranslationalSpeedMps()
        );
        SmartDashboard.putNumber(
            "ModoReto/ExpoTranslacao",
            Constants.ModoReto.EXPO_TRANSLACAO
        );
        SmartDashboard.putNumber(
            "ModoReto/HeadingUsadoPeloPIDDeg",
            drive.getRotation().getDegrees()
        );

        Logger.recordOutput("Drive/ModoReto/Enabled", modoReto.estaLigado());
        Logger.recordOutput(
            "Drive/ModoReto/VelocidadeMaximaMps",
            Constants.ModoReto.VELOCIDADE_TRANSLACAO_MAXIMA_MPS
        );
        Logger.recordOutput(
            "Drive/ModoReto/VelocidadeMinimaMps",
            Constants.ModoReto.VELOCIDADE_TRANSLACAO_MINIMA_MPS
        );
        Logger.recordOutput(
            "Drive/ModoReto/VelocidadeComandadaMps",
            Math.hypot(estado.ultimoVxField, estado.ultimoVyField)
        );
        Logger.recordOutput(
            "Drive/ModoReto/VelocidadeMedidaMps",
            drive.getTranslationalSpeedMps()
        );
        Logger.recordOutput("Drive/ModoReto/Estado", modoReto.getEstado().name());
        Logger.recordOutput(
            "Drive/ModoReto/HeadingRelativoDeg",
            drive.getRotation().getDegrees()
        );
        Logger.recordOutput(
            "Drive/ModoReto/DentroJanela20Deg",
            modoReto.deveSnaparZero(drive.getRotation())
        );
        Logger.recordOutput(
            "Drive/Input/SnapLongitudinal",
            inputProcessor.isSnappingLongitudinal()
        );
        Logger.recordOutput(
            "Drive/Input/SnapLateral",
            inputProcessor.isSnappingLateral()
        );
        Logger.recordOutput(
            "Drive/Input/ProcessedLongitudinal",
            processedTranslation.getX()
        );
        Logger.recordOutput(
            "Drive/Input/ProcessedLateral",
            processedTranslation.getY()
        );
    }
}
