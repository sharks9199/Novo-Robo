package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Robot-wide constants.
 *
 * <p>This class should not be used for any logic; it only holds numeric, boolean and string values
 * grouped by subsystem. Hardware calibration values (encoder offsets, sensor mounting) live in
 * {@link OffsetsConstants}.
 */
public final class Constants {

  // ---------------------------------------------------------------------------
  // Runtime mode
  // ---------------------------------------------------------------------------

  /**
   * Mode used when the code is not running on a real roboRIO. Change to {@code REPLAY} to replay
   * logs.
   */
  public static final Mode MODO_SIMULACAO = Mode.SIM;

  /** Current runtime mode: {@code REAL} on a roboRIO, otherwise {@link #MODO_SIMULACAO}. */
  public static final Mode MODO_ATUAL = RobotBase.isReal() ? Mode.REAL : MODO_SIMULACAO;

  /** Robot runtime modes. */
  public enum Mode {
    /** Running on a real robot. */
    REAL,
    /** Running a physics simulation. */
    SIM,
    /** Replaying a log file. */
    REPLAY
  }

  // ---------------------------------------------------------------------------
  // Driver Station
  // ---------------------------------------------------------------------------

  /** Driver Station controller ports. */
  public static final class Controles {
    public static final int PORTA_DRIVER = 0;
    public static final int PORTA_OPERADOR = 1;

    private Controles() {}
  }

  // ---------------------------------------------------------------------------
  // Swerve drivetrain
  // ---------------------------------------------------------------------------

  /** Swerve drivetrain physical properties, limits, gains and module definitions. */
  public static final class Swerve {

    // ----- Physical properties -----

    /** Robot mass including battery and bumpers, in kilograms. */
    public static final double MASSA_COM_BATERIA_E_BUMPERS_KG = 34.3;

    /** Distance between the centers of adjacent wheels, in meters (21 in). */
    public static final double DISTANCIA_ENTRE_RODAS_METROS = Units.inchesToMeters(21.0);

    /** Half of {@link #DISTANCIA_ENTRE_RODAS_METROS}; each module's offset from the center. */
    public static final double MEIA_DISTANCIA_ENTRE_RODAS_METROS =
        DISTANCIA_ENTRE_RODAS_METROS / 2.0;

    /** Estimated robot moment of inertia about the vertical axis, in kg*m^2. */
    public static final double MOMENTO_INERCIA_ESTIMADO_KG_M2 = 2.491;

    // ----- Current limits -----

    /** Drive motor stator current limit, in amps. */
    public static final double LIMITE_CORRENTE_TRACAO_AMPERES = 60.0;

    /** Steer motor stator current limit, in amps. */
    public static final double LIMITE_CORRENTE_DIRECAO_AMPERES = 40.0;

    // ----- Motion limits -----

    /** Maximum translational speed, in m/s. */
    public static final double VELOCIDADE_MAXIMA_MPS = 4.02;

    /** Maximum translational acceleration, in m/s^2. */
    public static final double ACELERACAO_MAXIMA_MPS2 = 7.5;

    /** Maximum rotational speed, in rad/s. */
    public static final double VELOCIDADE_ANGULAR_MAXIMA_RAD_S = Units.degreesToRadians(540.0);

    /** Maximum rotational acceleration, in rad/s^2. */
    public static final double ACELERACAO_ANGULAR_MAXIMA_RAD_S2 = Units.degreesToRadians(720.0);

    /** Maximum translational speed in slow mode, in m/s. */
    public static final double VELOCIDADE_MAXIMA_MPS_LENTO = 2.0;

    /** Maximum translational acceleration in slow mode, in m/s^2. */
    public static final double ACELERACAO_MAXIMA_MPS_LENTO = 1.0;

    // ----- Control loop -----

    /** Joystick deadband applied to the rotation axis. */
    public static final double DEADBAND_JOYSTICK = 0.03;

    /** Robot control loop period, in seconds. */
    public static final double PERIODO_CONTROLE_SEGUNDOS = 0.02;

    // ----- Closed-loop gains -----

    private static final Slot0Configs GANHOS_DIRECAO =
        new Slot0Configs()
            .withKP(25.0)
            .withKI(0.0)
            .withKD(2.5)
            .withKS(0.0)
            .withKV(0.12)
            .withKA(0.0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    private static final Slot0Configs GANHOS_TRACAO =
        new Slot0Configs().withKP(0.2).withKI(0.0).withKD(0.08).withKS(0.24).withKV(0.12);

    /*
     * Previous copy of the steer/drive gains, kept for reference.
     *
     * private static final Slot0Configs GANHOS_DIRECAO =
     *     new Slot0Configs()
     *         .withKP(25.0)
     *         .withKI(0.0)
     *         .withKD(2.5)
     *         .withKS(0.0)
     *         .withKV(0.12)
     *         .withKA(0.0)
     *         .withStaticFeedforwardSign(
     *             StaticFeedforwardSignValue.UseClosedLoopSign
     *         );
     *
     * private static final Slot0Configs GANHOS_TRACAO =
     *     new Slot0Configs()
     *         .withKP(0.2)
     *         .withKI(0.0)
     *         .withKD(0.08)
     *         .withKS(0.24)
     *         .withKV(0.12);
     */

    // ----- Module hardware configuration -----

    private static final ClosedLoopOutputType SAIDA_DIRECAO = ClosedLoopOutputType.Voltage;

    private static final ClosedLoopOutputType SAIDA_TRACAO = ClosedLoopOutputType.Voltage;

    private static final DriveMotorArrangement MOTOR_TRACAO =
        DriveMotorArrangement.TalonFX_Integrated;

    private static final SteerMotorArrangement MOTOR_DIRECAO =
        SteerMotorArrangement.TalonFX_Integrated;

    private static final SteerFeedbackType SENSOR_DIRECAO = SteerFeedbackType.FusedCANcoder;

    /** Stator current at which the wheel is assumed to slip. */
    private static final Current CORRENTE_DE_ESCORREGAMENTO =
        Amps.of(LIMITE_CORRENTE_TRACAO_AMPERES);

    private static final TalonFXConfiguration CONFIGURACAO_TRACAO = new TalonFXConfiguration();

    private static final TalonFXConfiguration CONFIGURACAO_DIRECAO =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(LIMITE_CORRENTE_DIRECAO_AMPERES))
                    .withStatorCurrentLimitEnable(true));

    private static final CANcoderConfiguration CONFIGURACAO_ENCODER = new CANcoderConfiguration();

    /** Pigeon 2 configuration, including its mounting orientation. */
    public static final Pigeon2Configuration CONFIGURACAO_PIGEON =
        new Pigeon2Configuration()
            .withMountPose(
                new MountPoseConfigs()
                    .withMountPoseYaw(OffsetsConstants.Pigeon.YAW)
                    .withMountPosePitch(OffsetsConstants.Pigeon.PITCH)
                    .withMountPoseRoll(OffsetsConstants.Pigeon.ROLL));

    /** Theoretical free speed of the drivetrain at 12 V. */
    public static final LinearVelocity VELOCIDADE_FISICA_A_12V = MetersPerSecond.of(4.20);

    // ----- Mechanical ratios and geometry -----

    /** Coupling ratio between the steer and drive shafts. */
    private static final double RELACAO_ACOPLAMENTO = 25.0 / 7.0;

    /** Drive motor to wheel gear reduction. */
    private static final double REDUCAO_TRACAO = 6.75;

    /** Steer motor to module gear reduction. */
    private static final double REDUCAO_DIRECAO = 150.0 / 7.0;

    /**
     * Wheel radius, computed from a 0.0964 m diameter.
     *
     * <p>NOTE: the original inline comment stated "96.8 mm = 48.4 mm", which does not match this
     * value. Verify the real wheel diameter.
     */
    private static final Distance RAIO_RODA = Meters.of(0.0964 / 2);

    // ----- Inversion and CAN IDs -----

    private static final boolean INVERTER_LADO_ESQUERDO = true;
    private static final boolean INVERTER_LADO_DIREITO = false;

    /** CAN ID of the Pigeon 2. */
    private static final int ID_PIGEON = 22;

    // ----- Simulation parameters -----

    private static final MomentOfInertia INERCIA_DIRECAO = KilogramSquareMeters.of(0.004);

    private static final MomentOfInertia INERCIA_TRACAO = KilogramSquareMeters.of(0.020);

    private static final Voltage ATRITO_DIRECAO = Volts.of(0.2);

    private static final Voltage ATRITO_TRACAO = Volts.of(0.25);

    // ----- Drivetrain and CAN bus -----

    /** Drivetrain-level constants (CAN bus name, Pigeon 2 ID and configuration). */
    public static final SwerveDrivetrainConstants DRIVETRAIN =
        new SwerveDrivetrainConstants()
            .withPigeon2Id(ID_PIGEON)
            .withPigeon2Configs(CONFIGURACAO_PIGEON);

    /** CAN bus the drivetrain devices are connected to. */
    public static final CANBus CAN_BUS = new CANBus(DRIVETRAIN.CANBusName);

    /** Factory that stamps out per-module constants sharing the same mechanical configuration. */
    private static final SwerveModuleConstantsFactory<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        FABRICA_MODULOS =
            new SwerveModuleConstantsFactory<
                    TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                .withDriveMotorGearRatio(REDUCAO_TRACAO)
                .withSteerMotorGearRatio(REDUCAO_DIRECAO)
                .withCouplingGearRatio(RELACAO_ACOPLAMENTO)
                .withWheelRadius(RAIO_RODA)
                .withSteerMotorGains(GANHOS_DIRECAO)
                .withDriveMotorGains(GANHOS_TRACAO)
                .withSteerMotorClosedLoopOutput(SAIDA_DIRECAO)
                .withDriveMotorClosedLoopOutput(SAIDA_TRACAO)
                .withSlipCurrent(CORRENTE_DE_ESCORREGAMENTO)
                .withSpeedAt12Volts(VELOCIDADE_FISICA_A_12V)
                .withDriveMotorType(MOTOR_TRACAO)
                .withSteerMotorType(MOTOR_DIRECAO)
                .withFeedbackSource(SENSOR_DIRECAO)
                .withDriveMotorInitialConfigs(CONFIGURACAO_TRACAO)
                .withSteerMotorInitialConfigs(CONFIGURACAO_DIRECAO)
                .withEncoderInitialConfigs(CONFIGURACAO_ENCODER)
                .withSteerInertia(INERCIA_DIRECAO)
                .withDriveInertia(INERCIA_TRACAO)
                .withSteerFrictionVoltage(ATRITO_DIRECAO)
                .withDriveFrictionVoltage(ATRITO_TRACAO);

    // ----- Module definitions -----

    /** Distance of each module from the robot center along each axis. */
    private static final Distance MEIA_DISTANCIA_MODULOS =
        Meters.of(MEIA_DISTANCIA_ENTRE_RODAS_METROS);

    /** Front-left module (steer 2, drive 1, encoder 9). */
    public static final SwerveModuleConstants<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        FRONT_LEFT =
            criarModulo(
                2,
                1,
                9,
                OffsetsConstants.Swerve.FRENTE_ESQUERDA,
                MEIA_DISTANCIA_MODULOS,
                MEIA_DISTANCIA_MODULOS,
                INVERTER_LADO_ESQUERDO,
                true,
                false);

    /** Front-right module (steer 4, drive 3, encoder 10). */
    public static final SwerveModuleConstants<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        FRONT_RIGHT =
            criarModulo(
                4,
                3,
                10,
                OffsetsConstants.Swerve.FRENTE_DIREITA,
                MEIA_DISTANCIA_MODULOS,
                MEIA_DISTANCIA_MODULOS.unaryMinus(),
                INVERTER_LADO_DIREITO,
                true,
                false);

    /** Back-left module (steer 6, drive 5, encoder 11). */
    public static final SwerveModuleConstants<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        BACK_LEFT =
            criarModulo(
                6,
                5,
                11,
                OffsetsConstants.Swerve.TRAS_ESQUERDA,
                MEIA_DISTANCIA_MODULOS.unaryMinus(),
                MEIA_DISTANCIA_MODULOS,
                INVERTER_LADO_ESQUERDO,
                true,
                false);

    /** Back-right module (steer 8, drive 7, encoder 12). */
    public static final SwerveModuleConstants<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        BACK_RIGHT =
            criarModulo(
                8,
                7,
                12,
                OffsetsConstants.Swerve.TRAS_DIREITA,
                MEIA_DISTANCIA_MODULOS.unaryMinus(),
                MEIA_DISTANCIA_MODULOS.unaryMinus(),
                INVERTER_LADO_DIREITO,
                true,
                false);

    /**
     * Creates the constants for a single module from the shared factory.
     *
     * @param idDirecao CAN ID of the steer motor
     * @param idTracao CAN ID of the drive motor
     * @param idEncoder CAN ID of the CANcoder
     * @param offsetEncoder CANcoder magnet offset
     * @param posicaoX module X position relative to the robot center (forward is positive)
     * @param posicaoY module Y position relative to the robot center (left is positive)
     * @param inverterTracao whether the drive motor is inverted
     * @param inverterDirecao whether the steer motor is inverted
     * @param inverterEncoder whether the CANcoder is inverted
     * @return the module constants
     */
    private static SwerveModuleConstants<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        criarModulo(
            int idDirecao,
            int idTracao,
            int idEncoder,
            Angle offsetEncoder,
            Distance posicaoX,
            Distance posicaoY,
            boolean inverterTracao,
            boolean inverterDirecao,
            boolean inverterEncoder) {
      return FABRICA_MODULOS.createModuleConstants(
          idDirecao,
          idTracao,
          idEncoder,
          offsetEncoder,
          posicaoX,
          posicaoY,
          inverterTracao,
          inverterDirecao,
          inverterEncoder);
    }

    private Swerve() {}
  }

  // ---------------------------------------------------------------------------
  // Driver input shaping
  // ---------------------------------------------------------------------------

  /** Joystick shaping parameters for teleoperated driving. */
  public static final class ControlesDrive {
    /** Radial deadband applied to the translation stick. */
    public static final double DEADBAND_RADIAL = 0.07;

    /** Exponent applied to the translation magnitude for finer control at low speeds. */
    public static final double EXPO_TRANSLACAO = 3;

    /** Minimum value of the dominant axis required to engage axis snapping. */
    public static final double EIXO_PRINCIPAL_MINIMO_SNAP = 0.18;

    /** Secondary axis must be below this value to engage axis snapping. */
    public static final double EIXO_SECUNDARIO_ENTRAR_SNAP = 0.11;

    /** Secondary axis must exceed this value to leave axis snapping (hysteresis). */
    public static final double EIXO_SECUNDARIO_SAIR_SNAP = 0.14;

    private ControlesDrive() {}
  }

  // ---------------------------------------------------------------------------
  // Heading hold
  // ---------------------------------------------------------------------------

  /** Heading-hold controller parameters. */
  public static final class Heading {
    public static final double KP = 30.0;
    public static final double KI = 0.0;
    public static final double KD = 10.0;

    /** Maximum rotational speed commanded by the heading correction, in rad/s. */
    public static final double VELOCIDADE_CORRECAO_MAXIMA_RAD_S = Units.degreesToRadians(120.0);

    /** Maximum rotational acceleration commanded by the heading correction, in rad/s^2. */
    public static final double ACELERACAO_CORRECAO_MAXIMA_RAD_S2 = Units.degreesToRadians(540.0);

    /** Position error below which the heading is considered reached, in radians. */
    public static final double TOLERANCIA_POSICAO_RAD = Units.degreesToRadians(0.1);

    /** Velocity below which the heading is considered settled, in rad/s. */
    public static final double TOLERANCIA_VELOCIDADE_RAD_S = Units.degreesToRadians(2.0);

    private Heading() {}
  }

  // ---------------------------------------------------------------------------
  // Autonomous
  // ---------------------------------------------------------------------------

  /** PathPlanner path-following parameters. */
  public static final class Autonomo {
    /** Proportional gain for translation tracking. */
    public static final double KP_TRANSLACAO = 4.0;

    /** Proportional gain for rotation tracking. */
    public static final double KP_ROTACAO = 4.0;

    /** Wheel coefficient of friction against the carpet. */
    public static final double COEFICIENTE_ATRITO_RODA = 1.15;

    private Autonomo() {}
  }

  // ---------------------------------------------------------------------------
  // Vision
  // ---------------------------------------------------------------------------

  /** Limelight / AprilTag vision parameters. */
  public static final class Vision {
    /** NetworkTables name of the Limelight camera. */
    public static final String NOME_LIMELIGHT = "limelight-sharks";

    // ----- Measurement acceptance -----

    /** Measurements from tags farther than this are rejected, in meters. */
    public static final double DISTANCIA_MAXIMA_VISAO_METROS = 6.0;

    /** Measurements are rejected while the robot rotates faster than this, in rad/s. */
    public static final double VELOCIDADE_ANGULAR_MAXIMA_PARA_VISAO_RAD_S =
        Units.degreesToRadians(540.0);

    /** Maximum distance at which a single tag may be used for heading, in meters. */
    public static final double DISTANCIA_MAXIMA_TAG_UNICA_HEADING_METROS = 2.5;

    /** Maximum pose ambiguity at which a single tag may be used for heading. */
    public static final double AMBIGUIDADE_MAXIMA_TAG_UNICA_HEADING = 0.15;

    // ----- MegaTag 1 standard deviations -----

    public static final double MT1_DESVIO_XY_MULTIPLAS_TAGS_METROS = 0.45;
    public static final double MT1_DESVIO_XY_TAG_UNICA_METROS = 0.80;
    public static final double MT1_DESVIO_THETA_MULTIPLAS_TAGS_RAD = Units.degreesToRadians(4.0);
    public static final double MT1_DESVIO_THETA_TAG_UNICA_RAD = Units.degreesToRadians(10.0);

    // ----- MegaTag 2 standard deviations -----

    public static final double MT2_DESVIO_XY_MULTIPLAS_TAGS_METROS = 0.30;
    public static final double MT2_DESVIO_XY_TAG_UNICA_METROS = 0.70;

    /** Effectively infinite theta deviation: MegaTag 2 must never correct heading. */
    public static final double MT2_DESVIO_THETA_RAD = 1_000_000.0;

    private Vision() {}
  }

  private Constants() {}
}
