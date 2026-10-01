package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.control.DirectionalInputProcessor;
import frc.robot.subsystems.drive.control.HeadingController;
import frc.robot.subsystems.drive.control.TranslationSlewLimiter;

public final class DriveCommands {
    private DriveCommands() {}

    public static Command joystickDrive(
        Drive drive,
        HeadingController headingController,
        DoubleSupplier frenteTras,
        DoubleSupplier lateral,
        DoubleSupplier rotacao
    ) {
        DirectionalInputProcessor inputProcessor = new DirectionalInputProcessor();

        TranslationSlewLimiter translationLimiter = new TranslationSlewLimiter(
            Constants.Chassi.ACELERACAO_MAXIMA_MPS2,
            Constants.Chassi.PERIODO_CONTROLE_SEGUNDOS
        );
        SlewRateLimiter omegaLimiter = new SlewRateLimiter(
            Constants.Chassi.ACELERACAO_ANGULAR_MAXIMA_RAD_S2
        );

        return Commands.run(
            () -> {
                double rawLongitudinal = frenteTras.getAsDouble();
                double rawLateral = lateral.getAsDouble();
                double rawRotation = rotacao.getAsDouble();

                Translation2d translation = inputProcessor.process(
                    rawLongitudinal,
                    rawLateral
                );
                boolean transladando = translation.getNorm() > 1e-6;

                double rotationInput = MathUtil.applyDeadband(
                    rawRotation,
                    Constants.Chassi.DEADBAND_JOYSTICK
                );
                boolean giroManual = Math.abs(rotationInput) > 1e-6;
                rotationInput = Math.copySign(
                    rotationInput * rotationInput,
                    rotationInput
                );

                double targetOmega;
                if (giroManual) {
                    headingController.desativar();
                    targetOmega = rotationInput
                        * Constants.Chassi.VELOCIDADE_ANGULAR_MAXIMA_RAD_S;
                } else if (!transladando) {
                    headingController.desativar();
                    targetOmega = 0.0;
                } else {
                    if (!headingController.estaAtivo()) {
                        if (
                            Math.abs(drive.getYawVelocityRadPerSec())
                                <= Constants.Heading.TOLERANCIA_VELOCIDADE_RAD_S
                        ) {
                            headingController.segurarHeadingAtual(
                                drive.getHeading(),
                                drive.getYawVelocityRadPerSec()
                            );
                        }
                    }
                    targetOmega = headingController.estaAtivo()
                        ? headingController.calcular(drive.getHeading())
                        : 0.0;
                }

                Translation2d targetVelocity = translation.times(
                    Constants.Chassi.VELOCIDADE_MAXIMA_MPS
                );

                Translation2d velocity = translationLimiter.calculate(targetVelocity);
                double vx = velocity.getX();
                double vy = velocity.getY();
                double omega = omegaLimiter.calculate(targetOmega);

                drive.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        vx,
                        vy,
                        omega,
                        drive.getRotation()
                    )
                );

                headingController.atualizarDashboard(drive.getHeading(), omega);

                Rotation2d ghostHeading = headingController.estaAtivo()
                    ? headingController.getHeadingAlvo()
                    : drive.getHeading();
                Logger.recordOutput(
                    "HeadingController/Ghost",
                    new Pose2d(drive.getPose().getTranslation(), ghostHeading)
                );

                SmartDashboard.putBoolean("DriveInput/GiroManual", giroManual);
                SmartDashboard.putNumber("DriveInput/Longitudinal", translation.getX());
                SmartDashboard.putNumber("DriveInput/Lateral", translation.getY());
                SmartDashboard.putNumber(
                    "DriveInput/VelocidadeComandadaMps",
                    velocity.getNorm()
                );
            },
            drive
        ).finallyDo(interrupted -> {
            headingController.desativar();
            inputProcessor.reset();
            translationLimiter.reset(Translation2d.kZero);
            omegaLimiter.reset(0.0);
            drive.stop();
        });
    }
}
