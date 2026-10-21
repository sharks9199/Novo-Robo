package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.Drive;
import frc.robot.subsystems.swerve.control.HeadingController;
import frc.robot.subsystems.swerve.control.SnapController;
import frc.robot.subsystems.swerve.control.TranslationSlewLimiter;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/** Factory methods for swerve drivetrain commands. */
public final class SwerveCommands {
  private SwerveCommands() {}

  /**
   * Creates the field-relative teleoperated drive command.
   *
   * <p>Behavior:
   *
   * <ul>
   *   <li>Translation input is shaped by a {@link SnapController} and rate-limited.
   *   <li>Manual rotation input has priority and disables heading hold.
   *   <li>When translating without rotation input, the heading is held by the {@link
   *       HeadingController}.
   *   <li>When the sticks are idle, the heading controller is disabled and the robot stops turning.
   * </ul>
   *
   * @param drive the drive subsystem
   * @param headingController heading-hold controller
   * @param frenteTras forward/backward axis, in [-1, 1] (forward positive)
   * @param lateral left/right axis, in [-1, 1] (left positive)
   * @param rotacao rotation axis, in [-1, 1] (counterclockwise positive)
   * @return the drive command; it never finishes on its own
   */
  public static Command movimentacaoDrive(
      Drive drive,
      HeadingController headingController,
      DoubleSupplier frenteTras,
      DoubleSupplier lateral,
      DoubleSupplier rotacao) {
    SnapController inputProcessor = new SnapController();

    TranslationSlewLimiter translationLimiter =
        new TranslationSlewLimiter(
            Constants.Swerve.ACELERACAO_MAXIMA_MPS2, Constants.Swerve.PERIODO_CONTROLE_SEGUNDOS);
    SlewRateLimiter omegaLimiter =
        new SlewRateLimiter(Constants.Swerve.ACELERACAO_ANGULAR_MAXIMA_RAD_S2);

    return Commands.run(
            () -> {
              // ----- Read and shape inputs -----
              double rawLongitudinal = frenteTras.getAsDouble();
              double rawLateral = lateral.getAsDouble();
              double rawRotation = rotacao.getAsDouble();

              Translation2d translation = inputProcessor.process(rawLongitudinal, rawLateral);
              boolean transladando = translation.getNorm() > 1e-6;

              double rotationInput =
                  MathUtil.applyDeadband(rawRotation, Constants.Swerve.DEADBAND_JOYSTICK);
              boolean giroManual = Math.abs(rotationInput) > 1e-6;
              // Square the input (keeping its sign) for finer control near the center.
              rotationInput = Math.copySign(rotationInput * rotationInput, rotationInput);

              // ----- Choose the angular velocity source -----
              double targetOmega;
              if (giroManual) {
                // Driver is rotating manually: release heading hold.
                headingController.desativar();
                targetOmega = rotationInput * Constants.Swerve.VELOCIDADE_ANGULAR_MAXIMA_RAD_S;
              } else if (!transladando) {
                // Robot is idle: nothing to hold.
                headingController.desativar();
                targetOmega = 0.0;
              } else {
                // Translating without rotation input: engage heading hold once the robot
                // has stopped rotating, so we latch onto a stable heading.
                if (!headingController.estaAtivo()) {
                  if (Math.abs(drive.getYawVelocityRadPerSec())
                      <= Constants.Heading.TOLERANCIA_VELOCIDADE_RAD_S) {
                    headingController.segurarHeadingAtual(
                        drive.getHeading(), drive.getYawVelocityRadPerSec());
                  }
                }
                targetOmega =
                    headingController.estaAtivo()
                        ? headingController.calcular(drive.getHeading())
                        : 0.0;
              }

              // ----- Apply acceleration limits -----
              Translation2d targetVelocity =
                  translation.times(Constants.Swerve.VELOCIDADE_MAXIMA_MPS);

              Translation2d velocity = translationLimiter.calculate(targetVelocity);
              double vx = velocity.getX();
              double vy = velocity.getY();
              double omega = omegaLimiter.calculate(targetOmega);

              // ----- Command the drivetrain (field-relative) -----
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, drive.getRotation()));

              // ----- Telemetry -----
              headingController.atualizarDashboard(drive.getHeading(), omega);

              // "Ghost" pose showing where the heading controller is trying to point.
              Rotation2d ghostHeading =
                  headingController.estaAtivo()
                      ? headingController.getHeadingAlvo()
                      : drive.getHeading();
              Logger.recordOutput(
                  "HeadingController/Ghost",
                  new Pose2d(drive.getPose().getTranslation(), ghostHeading));

              SmartDashboard.putBoolean("DriveInput/GiroManual", giroManual);
              SmartDashboard.putNumber("DriveInput/Longitudinal", translation.getX());
              SmartDashboard.putNumber("DriveInput/Lateral", translation.getY());
              SmartDashboard.putNumber("DriveInput/VelocidadeComandadaMps", velocity.getNorm());
            },
            drive)
        .finallyDo(
            interrupted -> {
              // Leave the drivetrain in a clean, stopped state.
              headingController.desativar();
              inputProcessor.reset();
              translationLimiter.reset(Translation2d.kZero);
              omegaLimiter.reset(0.0);
              drive.stop();
            });
  }
}
