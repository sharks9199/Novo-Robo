package frc.robot.subsystems.drive.control;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public final class HeadingController {
    private static final String DASHBOARD = "HeadingTuning/";//quero "tunar"

    private final ProfiledPIDController controller = new ProfiledPIDController(
        Constants.Heading.KP,
        Constants.Heading.KI,
        Constants.Heading.KD,
        new TrapezoidProfile.Constraints(
            Constants.Heading.VELOCIDADE_CORRECAO_MAXIMA_RAD_S,
            Constants.Heading.ACELERACAO_CORRECAO_MAXIMA_RAD_S2
        ),//recomendação do silva
        Constants.Chassi.PERIODO_CONTROLE_SEGUNDOS//0.02
    );

    private Rotation2d headingAlvo = Rotation2d.kZero;
    private boolean ativo = false;

    private double kP = Constants.Heading.KP;
    private double kI = Constants.Heading.KI;
    private double kD = Constants.Heading.KD;
    private double velocidadeCorrecaoMaximaRadS =
        Constants.Heading.VELOCIDADE_CORRECAO_MAXIMA_RAD_S;
    private double aceleracaoCorrecaoMaximaRadS2 =
        Constants.Heading.ACELERACAO_CORRECAO_MAXIMA_RAD_S2;
    private double toleranciaPosicaoRad =
        Constants.Heading.TOLERANCIA_POSICAO_RAD;
    private double toleranciaVelocidadeRadS =
        Constants.Heading.TOLERANCIA_VELOCIDADE_RAD_S;
    private double ultimoOmegaPidRadS = 0.0;
    private double ultimoErroRad = 0.0;

    public HeadingController() {
        controller.enableContinuousInput(-Math.PI, Math.PI);
        controller.setTolerance(toleranciaPosicaoRad, toleranciaVelocidadeRadS);
        //para eu lembra: precisei usar os dois argumentos. Um é a tolerancia da posição e o outro é da velocidade.

        SmartDashboard.setDefaultNumber(DASHBOARD + "kP", kP);
        SmartDashboard.setDefaultNumber(DASHBOARD + "kI", kI);
        SmartDashboard.setDefaultNumber(DASHBOARD + "kD", kD);
        SmartDashboard.setDefaultNumber(
            DASHBOARD + "MaxCorrectionDegPerSec",
            Units.radiansToDegrees(velocidadeCorrecaoMaximaRadS)
        );
        SmartDashboard.setDefaultNumber(
            DASHBOARD + "MaxCorrectionAccelDegPerSec2",
            Units.radiansToDegrees(aceleracaoCorrecaoMaximaRadS2)
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

    public void segurarHeadingAtual(Rotation2d headingAtual, double yawVelocityRadS) {
        definirHeading(headingAtual, headingAtual, yawVelocityRadS);
    }

    public void segurarHeadingAtual(Rotation2d headingAtual) {
        segurarHeadingAtual(headingAtual, 0.0);
    }

    public void definirHeading(
        Rotation2d novoHeading,
        Rotation2d headingAtual,
        double yawVelocityRadS
    ) {
        headingAlvo = novoHeading;
        ultimoErroRad = MathUtil.angleModulus(
            headingAlvo.getRadians() - headingAtual.getRadians()
        );
        controller.reset(headingAtual.getRadians(), yawVelocityRadS);
        controller.setGoal(headingAlvo.getRadians());
        ativo = true;
    }

    public void definirHeading(Rotation2d novoHeading) {
        headingAlvo = novoHeading;
        controller.setGoal(headingAlvo.getRadians());
        ativo = true;
    }

    public double calcular(Rotation2d headingAtual) {
        atualizarTuningDoElastic();

        if (!ativo) {
            ultimoErroRad = 0.0;
            ultimoOmegaPidRadS = 0.0;
            return 0.0;
        }

        ultimoErroRad = MathUtil.angleModulus(
            headingAlvo.getRadians() - headingAtual.getRadians()
        );

        if (
            Math.abs(ultimoErroRad) <= toleranciaPosicaoRad
            && Math.abs(controller.getSetpoint().velocity)
                <= toleranciaVelocidadeRadS
        ) {
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

        double erroAtualRad = ativo
            ? MathUtil.angleModulus(
                headingAlvo.getRadians() - headingAtual.getRadians()
            )
            : 0.0;

        if (ativo) {
            ultimoErroRad = erroAtualRad;
        }

        SmartDashboard.putBoolean(DASHBOARD + "Active", ativo);
        SmartDashboard.putBoolean(DASHBOARD + "AtSetpoint", estaNoAlvo());
        SmartDashboard.putNumber(
            DASHBOARD + "CurrentHeadingDeg",
            headingAtual.getDegrees()
        );
        SmartDashboard.putNumber(
            DASHBOARD + "TargetHeadingDeg",
            headingAlvo.getDegrees()
        );
        SmartDashboard.putNumber(
            DASHBOARD + "ErrorDeg",
            Units.radiansToDegrees(erroAtualRad)
        );
        SmartDashboard.putNumber(
            DASHBOARD + "ProfileSetpointDeg",
            Units.radiansToDegrees(controller.getSetpoint().position)
        );
        SmartDashboard.putNumber(
            DASHBOARD + "ProfileVelocityDegPerSec",
            Units.radiansToDegrees(controller.getSetpoint().velocity)
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
            30.0
        );
        double novoKI = MathUtil.clamp(
            SmartDashboard.getNumber(DASHBOARD + "kI", Constants.Heading.KI),
            0.0,
            5.0
        );
        double novoKD = MathUtil.clamp(
            SmartDashboard.getNumber(DASHBOARD + "kD", Constants.Heading.KD),
            0.0,
            10.0
        );

        if (novoKP != kP || novoKI != kI || novoKD != kD) { // normalmente eu usava OR (python), mas em java é ||
            kP = novoKP;
            kI = novoKI;
            kD = novoKD;
            controller.setPID(kP, kI, kD);
        }

        double maxCorrecaoDegS = MathUtil.clamp(
            SmartDashboard.getNumber(
                DASHBOARD + "MaxCorrectionDegPerSec",
                Units.radiansToDegrees(
                    Constants.Heading.VELOCIDADE_CORRECAO_MAXIMA_RAD_S
                )
            ),
            1.0,
            720.0
        );
        double maxAccelDegS2 = MathUtil.clamp(
            SmartDashboard.getNumber(
                DASHBOARD + "MaxCorrectionAccelDegPerSec2",
                Units.radiansToDegrees(
                    Constants.Heading.ACELERACAO_CORRECAO_MAXIMA_RAD_S2
                )
            ),
            10.0,
            5000.0
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
                Units.radiansToDegrees(
                    Constants.Heading.TOLERANCIA_VELOCIDADE_RAD_S
                )
            ),
            0.1,
            180.0
        );

        double novaVelMax = Units.degreesToRadians(maxCorrecaoDegS);
        double novaAccelMax = Units.degreesToRadians(maxAccelDegS2);
        if (
            novaVelMax != velocidadeCorrecaoMaximaRadS
            || novaAccelMax != aceleracaoCorrecaoMaximaRadS2
        ) {
            velocidadeCorrecaoMaximaRadS = novaVelMax;
            aceleracaoCorrecaoMaximaRadS2 = novaAccelMax;
            controller.setConstraints(
                new TrapezoidProfile.Constraints(
                    velocidadeCorrecaoMaximaRadS,
                    aceleracaoCorrecaoMaximaRadS2
                )
            );
        }

        toleranciaPosicaoRad = Units.degreesToRadians(toleranciaPosicaoDeg);
        toleranciaVelocidadeRadS = Units.degreesToRadians(
            toleranciaVelocidadeDegS
        );
        controller.setTolerance(toleranciaPosicaoRad, toleranciaVelocidadeRadS);
    }

    public void aplicarCorrecaoReferencia(Rotation2d correcao) {
        if (!ativo || Math.abs(correcao.getRadians()) < 1e-9) {
            return;
        }
        headingAlvo = headingAlvo.plus(correcao);
        controller.setGoal(headingAlvo.getRadians());
    }

    public void desativar() {
        ativo = false;
        ultimoErroRad = 0.0;
        ultimoOmegaPidRadS = 0.0;
    }

    public boolean estaAtivo() {
        return ativo;
    }

    public boolean estaNoAlvo() {
        return ativo
            && Math.abs(ultimoErroRad) <= toleranciaPosicaoRad
            && Math.abs(controller.getSetpoint().velocity)
                <= toleranciaVelocidadeRadS;
    }

    public Rotation2d getHeadingAlvo() {
        return headingAlvo;
    }

    public double getErroRad() {
        return ativo ? ultimoErroRad : 0.0;
    }
}
