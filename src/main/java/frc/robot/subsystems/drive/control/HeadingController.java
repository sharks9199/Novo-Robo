package frc.robot.subsystems.drive.control;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public final class HeadingController {
    private static final String DASHBOARD = "HeadingTuning/";

    private final PIDController controller = new PIDController(
        Constants.Heading.KP,
        Constants.Heading.KI,
        Constants.Heading.KD,
        Constants.Chassi.PERIODO_CONTROLE_SEGUNDOS
    );

    private Rotation2d headingAlvo = Rotation2d.kZero;
    private boolean ativo = false;

    private double kP = Constants.Heading.KP;
    private double kI = Constants.Heading.KI;
    private double kD = Constants.Heading.KD;
    private double velocidadeCorrecaoMaximaRadS =
        Constants.Heading.VELOCIDADE_CORRECAO_MAXIMA_RAD_S;
    private double toleranciaPosicaoRad =
        Constants.Heading.TOLERANCIA_POSICAO_RAD;
    private double toleranciaVelocidadeRadS =
        Constants.Heading.TOLERANCIA_VELOCIDADE_RAD_S;
    private double ultimoOmegaPidRadS = 0.0;

    public HeadingController() {
        controller.enableContinuousInput(-Math.PI, Math.PI);
        controller.setTolerance(toleranciaPosicaoRad, toleranciaVelocidadeRadS);

        SmartDashboard.setDefaultNumber(DASHBOARD + "kP", kP);
        SmartDashboard.setDefaultNumber(DASHBOARD + "kI", kI);
        SmartDashboard.setDefaultNumber(DASHBOARD + "kD", kD);
        SmartDashboard.setDefaultNumber(
            DASHBOARD + "MaxCorrectionDegPerSec",
            Units.radiansToDegrees(velocidadeCorrecaoMaximaRadS)
        );
        SmartDashboard.setDefaultNumber(
            DASHBOARD + "PositionToleranceDeg",
            Units.radiansToDegrees(toleranciaPosicaoRad)
        );
        SmartDashboard.setDefaultNumber(
            DASHBOARD + "VelocityToleranceDegPerSec",
            Units.radiansToDegrees(toleranciaVelocidadeRadS)
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
        atualizarTuningDoElastic();

        if (!ativo) {
            ultimoOmegaPidRadS = 0.0;
            return 0.0;
        }

        double omega = controller.calculate(
            headingAtual.getRadians(),
            headingAlvo.getRadians()
        );

        ultimoOmegaPidRadS = MathUtil.clamp(
            omega,
            -velocidadeCorrecaoMaximaRadS,
            velocidadeCorrecaoMaximaRadS
        );

        return ultimoOmegaPidRadS;
    }

    public void atualizarDashboard(
        Rotation2d headingAtual,
        double omegaAplicadoRadS
    ) {
        atualizarTuningDoElastic();

        SmartDashboard.putBoolean(DASHBOARD + "Active", ativo);
        SmartDashboard.putBoolean(DASHBOARD + "AtSetpoint", estaNoAlvo());
        SmartDashboard.putNumber(DASHBOARD + "CurrentHeadingDeg", headingAtual.getDegrees());
        SmartDashboard.putNumber(DASHBOARD + "TargetHeadingDeg", headingAlvo.getDegrees());
        SmartDashboard.putNumber(
            DASHBOARD + "ErrorDeg",
            ativo ? Units.radiansToDegrees(controller.getError()) : 0.0
        );
        SmartDashboard.putNumber(
            DASHBOARD + "PIDOutputDegPerSec",
            Units.radiansToDegrees(ultimoOmegaPidRadS)
        );
        SmartDashboard.putNumber(
            DASHBOARD + "AppliedOmegaDegPerSec",
            Units.radiansToDegrees(omegaAplicadoRadS)
        );
    }

    private void atualizarTuningDoElastic() {
        double novoKP = MathUtil.clamp(
            SmartDashboard.getNumber(DASHBOARD + "kP", Constants.Heading.KP),
            0.0,
            20.0
        );
        double novoKI = MathUtil.clamp(
            SmartDashboard.getNumber(DASHBOARD + "kI", Constants.Heading.KI),
            0.0,
            5.0
        );
        double novoKD = MathUtil.clamp(
            SmartDashboard.getNumber(DASHBOARD + "kD", Constants.Heading.KD),
            0.0,
            5.0
        );

        if (novoKP != kP || novoKI != kI || novoKD != kD) {
            kP = novoKP;
            kI = novoKI;
            kD = novoKD;
            controller.setPID(kP, kI, kD);
            controller.reset();
        }

        double maxCorrecaoDegS = MathUtil.clamp(
            SmartDashboard.getNumber(
                DASHBOARD + "MaxCorrectionDegPerSec",
                Units.radiansToDegrees(Constants.Heading.VELOCIDADE_CORRECAO_MAXIMA_RAD_S)
            ),
            0.0,
            720.0
        );

        double toleranciaPosicaoDeg = MathUtil.clamp(
            SmartDashboard.getNumber(
                DASHBOARD + "PositionToleranceDeg",
                Units.radiansToDegrees(Constants.Heading.TOLERANCIA_POSICAO_RAD)
            ),
            0.05,
            20.0
        );

        double toleranciaVelocidadeDegS = MathUtil.clamp(
            SmartDashboard.getNumber(
                DASHBOARD + "VelocityToleranceDegPerSec",
                Units.radiansToDegrees(Constants.Heading.TOLERANCIA_VELOCIDADE_RAD_S)
            ),
            0.1,
            180.0
        );

        velocidadeCorrecaoMaximaRadS = Units.degreesToRadians(maxCorrecaoDegS);
        toleranciaPosicaoRad = Units.degreesToRadians(toleranciaPosicaoDeg);
        toleranciaVelocidadeRadS = Units.degreesToRadians(toleranciaVelocidadeDegS);

        controller.setTolerance(toleranciaPosicaoRad, toleranciaVelocidadeRadS);
    }

    public void aplicarCorrecaoReferencia(Rotation2d correcao) {
        if (!ativo || Math.abs(correcao.getRadians()) < 1e-9) {
            return;
        }

        headingAlvo = headingAlvo.plus(correcao);
        controller.reset();
    }

    public void desativar() {
        ativo = false;
        ultimoOmegaPidRadS = 0.0;
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
        return ativo ? controller.getError() : 0.0;
    }
}
