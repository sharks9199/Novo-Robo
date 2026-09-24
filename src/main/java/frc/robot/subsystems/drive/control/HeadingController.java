package frc.robot.subsystems.drive.control;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public final class HeadingController {
    private final PIDController controller = new PIDController(
        Constants.Heading.KP,
        Constants.Heading.KI,
        Constants.Heading.KD,
        Constants.Chassi.PERIODO_CONTROLE_SEGUNDOS
    );

    private Rotation2d headingAlvo = Rotation2d.kZero;
    private boolean ativo = false;

    public HeadingController() {
        controller.enableContinuousInput(-Math.PI, Math.PI);
        controller.setTolerance(
            Constants.Heading.TOLERANCIA_POSICAO_RAD,
            Constants.Heading.TOLERANCIA_VELOCIDADE_RAD_S
        );
    }

    public void segurarHeadingAtual(Rotation2d headingAtual) {
        headingAlvo = headingAtual;
        controller.reset();
        ativo = true;
    }

    public void definirHeading(Rotation2d novoHeading) {
        headingAlvo = novoHeading;
        controller.reset();
        ativo = true;
    }

    public double calcular(Rotation2d headingAtual) {
        if (!ativo) {
            return 0.0;
        }

        double omega = controller.calculate(
            headingAtual.getRadians(),
            headingAlvo.getRadians()
        );

        return MathUtil.clamp(
            omega,
            -Constants.Heading.VELOCIDADE_CORRECAO_MAXIMA_RAD_S,
            Constants.Heading.VELOCIDADE_CORRECAO_MAXIMA_RAD_S
        );
    }

    public void desativar() {
        ativo = false;
        controller.reset();
    }

    public boolean estaAtivo() {
        return ativo;
    }

    public boolean estaNoAlvo() {
        return ativo && controller.atSetpoint();
    }

    public Rotation2d getHeadingAlvo() {
        return headingAlvo;
    }

    public double getErroRad() {
        return controller.getError();
    }
}
