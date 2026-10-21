package frc.robot.subsystems.swerve.control;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * Holds the robot's heading using a profiled PID controller.
 *
 * <p>The controller outputs an angular velocity (rad/s) that is added to the chassis speeds. Gains,
 * motion constraints and tolerances can be tuned live from the dashboard (Elastic) under the {@code
 * HeadingTuning/} table.
 */
public final class HeadingController {
  /** NetworkTables prefix for tuning and telemetry entries. */
  private static final String DASHBOARD = "HeadingTuning/";

  private final ProfiledPIDController controller =
      new ProfiledPIDController(
          Constants.Heading.KP,
          Constants.Heading.KI,
          Constants.Heading.KD,
          new TrapezoidProfile.Constraints(
              Constants.Heading.VELOCIDADE_CORRECAO_MAXIMA_RAD_S,
              Constants.Heading.ACELERACAO_CORRECAO_MAXIMA_RAD_S2), // Values recommended by Silva.
          Constants.Swerve.PERIODO_CONTROLE_SEGUNDOS); // 0.02 s

  private Rotation2d headingAlvo = Rotation2d.kZero;
  private boolean ativo = false;

  // Live-tunable values (mirrors of the dashboard entries).
  private double kP = Constants.Heading.KP;
  private double kI = Constants.Heading.KI;
  private double kD = Constants.Heading.KD;
  private double velocidadeCorrecaoMaximaRadS = Constants.Heading.VELOCIDADE_CORRECAO_MAXIMA_RAD_S;
  private double aceleracaoCorrecaoMaximaRadS2 =
      Constants.Heading.ACELERACAO_CORRECAO_MAXIMA_RAD_S2;
  private double toleranciaPosicaoRad = Constants.Heading.TOLERANCIA_POSICAO_RAD;
  private double toleranciaVelocidadeRadS = Constants.Heading.TOLERANCIA_VELOCIDADE_RAD_S;

  // Last computed values, used for telemetry and "at target" checks.
  private double ultimoOmegaPidRadS = 0.0;
  private double ultimoErroRad = 0.0;

  /** Creates the controller and publishes default tuning values to the dashboard. */
  public HeadingController() {
    controller.enableContinuousInput(-Math.PI, Math.PI);
    // Both arguments are required: the first is the position tolerance, the second the
    // velocity tolerance.
    controller.setTolerance(toleranciaPosicaoRad, toleranciaVelocidadeRadS);

    SmartDashboard.setDefaultNumber(DASHBOARD + "kP", kP);
    SmartDashboard.setDefaultNumber(DASHBOARD + "kI", kI);
    SmartDashboard.setDefaultNumber(DASHBOARD + "kD", kD);
    SmartDashboard.setDefaultNumber(
        DASHBOARD + "MaxCorrectionDegPerSec", Units.radiansToDegrees(velocidadeCorrecaoMaximaRadS));
    SmartDashboard.setDefaultNumber(
        DASHBOARD + "MaxCorrectionAccelDegPerSec2",
        Units.radiansToDegrees(aceleracaoCorrecaoMaximaRadS2));
    SmartDashboard.setDefaultNumber(
        DASHBOARD + "PositionToleranceDeg", Units.radiansToDegrees(toleranciaPosicaoRad));
    SmartDashboard.setDefaultNumber(
        DASHBOARD + "VelocityToleranceDegPerSec", Units.radiansToDegrees(toleranciaVelocidadeRadS));
  }

  // ---------------------------------------------------------------------------
  // Goal management
  // ---------------------------------------------------------------------------

  /**
   * Holds the current heading, resetting the profile to the measured state.
   *
   * @param headingAtual current robot heading
   * @param yawVelocityRadS current yaw rate, in rad/s
   */
  public void segurarHeadingAtual(Rotation2d headingAtual, double yawVelocityRadS) {
    definirHeading(headingAtual, headingAtual, yawVelocityRadS);
  }

  /**
   * Holds the current heading, assuming the robot is not rotating.
   *
   * @param headingAtual current robot heading
   */
  public void segurarHeadingAtual(Rotation2d headingAtual) {
    segurarHeadingAtual(headingAtual, 0.0);
  }

  /**
   * Sets a new heading goal and resets the motion profile to the measured state.
   *
   * @param novoHeading desired heading
   * @param headingAtual current robot heading
   * @param yawVelocityRadS current yaw rate, in rad/s
   */
  public void definirHeading(
      Rotation2d novoHeading, Rotation2d headingAtual, double yawVelocityRadS) {
    headingAlvo = novoHeading;
    ultimoErroRad = MathUtil.angleModulus(headingAlvo.getRadians() - headingAtual.getRadians());
    controller.reset(headingAtual.getRadians(), yawVelocityRadS);
    controller.setGoal(headingAlvo.getRadians());
    ativo = true;
  }

  /**
   * Sets a new heading goal without resetting the motion profile.
   *
   * @param novoHeading desired heading
   */
  public void definirHeading(Rotation2d novoHeading) {
    headingAlvo = novoHeading;
    controller.setGoal(headingAlvo.getRadians());
    ativo = true;
  }

  /**
   * Shifts the heading goal by a correction, e.g. after a vision update changed the pose rotation.
   * Has no effect while the controller is inactive.
   *
   * @param correcao rotation to add to the current goal
   */
  public void aplicarCorrecaoReferencia(Rotation2d correcao) {
    if (!ativo || Math.abs(correcao.getRadians()) < 1e-9) {
      return;
    }
    headingAlvo = headingAlvo.plus(correcao);
    controller.setGoal(headingAlvo.getRadians());
  }

  /** Deactivates the controller; {@link #calcular} returns zero until a new goal is set. */
  public void desativar() {
    ativo = false;
    ultimoErroRad = 0.0;
    ultimoOmegaPidRadS = 0.0;
  }

  // ---------------------------------------------------------------------------
  // Control output
  // ---------------------------------------------------------------------------

  /**
   * Computes the angular velocity needed to reach the heading goal.
   *
   * @param headingAtual current robot heading
   * @return angular velocity command, in rad/s; zero if inactive or already at the goal
   */
  public double calcular(Rotation2d headingAtual) {
    atualizarTuningDoElastic();

    if (!ativo) {
      ultimoErroRad = 0.0;
      ultimoOmegaPidRadS = 0.0;
      return 0.0;
    }

    ultimoErroRad = MathUtil.angleModulus(headingAlvo.getRadians() - headingAtual.getRadians());

    if (Math.abs(ultimoErroRad) <= toleranciaPosicaoRad
        && Math.abs(controller.getSetpoint().velocity) <= toleranciaVelocidadeRadS) {
      ultimoOmegaPidRadS = 0.0;
      return 0.0;
    }

    double omega = controller.calculate(headingAtual.getRadians(), headingAlvo.getRadians());

    ultimoOmegaPidRadS =
        MathUtil.clamp(omega, -velocidadeCorrecaoMaximaRadS, velocidadeCorrecaoMaximaRadS);
    return ultimoOmegaPidRadS;
  }

  // ---------------------------------------------------------------------------
  // Telemetry and tuning
  // ---------------------------------------------------------------------------

  /**
   * Publishes controller state to the dashboard.
   *
   * @param headingAtual current robot heading
   * @param omegaAplicadoRadS angular velocity actually applied to the chassis, in rad/s
   */
  public void atualizarDashboard(Rotation2d headingAtual, double omegaAplicadoRadS) {
    atualizarTuningDoElastic();

    double erroAtualRad =
        ativo
            ? MathUtil.angleModulus(headingAlvo.getRadians() - headingAtual.getRadians())
            : 0.0;

    if (ativo) {
      ultimoErroRad = erroAtualRad;
    }

    SmartDashboard.putBoolean(DASHBOARD + "Active", ativo);
    SmartDashboard.putBoolean(DASHBOARD + "AtSetpoint", estaNoAlvo());
    SmartDashboard.putNumber(DASHBOARD + "CurrentHeadingDeg", headingAtual.getDegrees());
    SmartDashboard.putNumber(DASHBOARD + "TargetHeadingDeg", headingAlvo.getDegrees());
    SmartDashboard.putNumber(DASHBOARD + "ErrorDeg", Units.radiansToDegrees(erroAtualRad));
    SmartDashboard.putNumber(
        DASHBOARD + "ProfileSetpointDeg",
        Units.radiansToDegrees(controller.getSetpoint().position));
    SmartDashboard.putNumber(
        DASHBOARD + "ProfileVelocityDegPerSec",
        Units.radiansToDegrees(controller.getSetpoint().velocity));
    SmartDashboard.putNumber(
        DASHBOARD + "PIDOutputDegPerSec", Units.radiansToDegrees(ultimoOmegaPidRadS));
    SmartDashboard.putNumber(
        DASHBOARD + "AppliedOmegaDegPerSec", Units.radiansToDegrees(omegaAplicadoRadS));
  }

  /**
   * Reads tuning values from the dashboard, clamps them to safe ranges and applies them to the
   * controller when they have changed.
   */
  private void atualizarTuningDoElastic() {
    // PID gains
    double novoKP =
        MathUtil.clamp(SmartDashboard.getNumber(DASHBOARD + "kP", Constants.Heading.KP), 0.0, 30.0);
    double novoKI =
        MathUtil.clamp(SmartDashboard.getNumber(DASHBOARD + "kI", Constants.Heading.KI), 0.0, 5.0);
    double novoKD =
        MathUtil.clamp(SmartDashboard.getNumber(DASHBOARD + "kD", Constants.Heading.KD), 0.0, 10.0);

    if (novoKP != kP || novoKI != kI || novoKD != kD) {
      kP = novoKP;
      kI = novoKI;
      kD = novoKD;
      controller.setPID(kP, kI, kD);
    }

    // Motion profile constraints and tolerances
    double maxCorrecaoDegS =
        MathUtil.clamp(
            SmartDashboard.getNumber(
                DASHBOARD + "MaxCorrectionDegPerSec",
                Units.radiansToDegrees(Constants.Heading.VELOCIDADE_CORRECAO_MAXIMA_RAD_S)),
            1.0,
            720.0);
    double maxAccelDegS2 =
        MathUtil.clamp(
            SmartDashboard.getNumber(
                DASHBOARD + "MaxCorrectionAccelDegPerSec2",
                Units.radiansToDegrees(Constants.Heading.ACELERACAO_CORRECAO_MAXIMA_RAD_S2)),
            10.0,
            5000.0);
    double toleranciaPosicaoDeg =
        MathUtil.clamp(
            SmartDashboard.getNumber(
                DASHBOARD + "PositionToleranceDeg",
                Units.radiansToDegrees(Constants.Heading.TOLERANCIA_POSICAO_RAD)),
            0.05,
            20.0);
    double toleranciaVelocidadeDegS =
        MathUtil.clamp(
            SmartDashboard.getNumber(
                DASHBOARD + "VelocityToleranceDegPerSec",
                Units.radiansToDegrees(Constants.Heading.TOLERANCIA_VELOCIDADE_RAD_S)),
            0.1,
            180.0);

    double novaVelMax = Units.degreesToRadians(maxCorrecaoDegS);
    double novaAccelMax = Units.degreesToRadians(maxAccelDegS2);
    if (novaVelMax != velocidadeCorrecaoMaximaRadS
        || novaAccelMax != aceleracaoCorrecaoMaximaRadS2) {
      velocidadeCorrecaoMaximaRadS = novaVelMax;
      aceleracaoCorrecaoMaximaRadS2 = novaAccelMax;
      controller.setConstraints(
          new TrapezoidProfile.Constraints(
              velocidadeCorrecaoMaximaRadS, aceleracaoCorrecaoMaximaRadS2));
    }

    toleranciaPosicaoRad = Units.degreesToRadians(toleranciaPosicaoDeg);
    toleranciaVelocidadeRadS = Units.degreesToRadians(toleranciaVelocidadeDegS);
    controller.setTolerance(toleranciaPosicaoRad, toleranciaVelocidadeRadS);
  }

  // ---------------------------------------------------------------------------
  // State accessors
  // ---------------------------------------------------------------------------

  /** Returns whether the controller currently holds a heading goal. */
  public boolean estaAtivo() {
    return ativo;
  }

  /** Returns whether the heading error and profile velocity are both within tolerance. */
  public boolean estaNoAlvo() {
    return ativo
        && Math.abs(ultimoErroRad) <= toleranciaPosicaoRad
        && Math.abs(controller.getSetpoint().velocity) <= toleranciaVelocidadeRadS;
  }

  /** Returns the current heading goal. */
  public Rotation2d getHeadingAlvo() {
    return headingAlvo;
  }

  /** Returns the last computed heading error in radians, or zero when inactive. */
  public double getErroRad() {
    return ativo ? ultimoErroRad : 0.0;
  }
}
