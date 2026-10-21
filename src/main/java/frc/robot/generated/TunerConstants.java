package frc.robot.generated;

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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.OffsetsConstants;

/**
 * Physical configuration of the four swerve modules, generated from Phoenix Tuner X.
 *
 * <p>The robot's operating limits live in {@link Constants.Swerve}. This class keeps the mechanical
 * ratios, CAN IDs, offsets and gains that must match the real hardware.
 */
public final class TunerConstants {

  // ---------------------------------------------------------------------------
  // Closed-loop gains
  // ---------------------------------------------------------------------------

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

  // ---------------------------------------------------------------------------
  // Module hardware configuration
  // ---------------------------------------------------------------------------

  private static final ClosedLoopOutputType SAIDA_DIRECAO = ClosedLoopOutputType.Voltage;
  private static final ClosedLoopOutputType SAIDA_TRACAO = ClosedLoopOutputType.Voltage;

  private static final DriveMotorArrangement MOTOR_TRACAO =
      DriveMotorArrangement.TalonFX_Integrated;
  private static final SteerMotorArrangement MOTOR_DIRECAO =
      SteerMotorArrangement.TalonFX_Integrated;
  private static final SteerFeedbackType SENSOR_DIRECAO = SteerFeedbackType.FusedCANcoder;

  /** Stator current at which the wheel is assumed to slip. */
  private static final Current CORRENTE_DE_ESCORREGAMENTO =
      Amps.of(Constants.Swerve.LIMITE_CORRENTE_TRACAO_AMPERES);

  private static final TalonFXConfiguration CONFIGURACAO_TRACAO = new TalonFXConfiguration();

  private static final TalonFXConfiguration CONFIGURACAO_DIRECAO =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(Amps.of(Constants.Swerve.LIMITE_CORRENTE_DIRECAO_AMPERES))
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

  /** Approximate physical speed at 12 V, used by the module model. */
  public static final LinearVelocity VELOCIDADE_FISICA_A_12V = MetersPerSecond.of(4.5);

  // ---------------------------------------------------------------------------
  // Mechanical ratios and geometry
  // ---------------------------------------------------------------------------

  private static final double RELACAO_ACOPLAMENTO = 25.0 / 7.0;
  private static final double REDUCAO_TRACAO = 6.75;
  private static final double REDUCAO_DIRECAO = 150.0 / 7.0;
  private static final Distance RAIO_RODA = Meters.of(0.048);

  // ---------------------------------------------------------------------------
  // Inversion and CAN IDs
  // ---------------------------------------------------------------------------

  private static final boolean INVERTER_LADO_ESQUERDO = true;
  private static final boolean INVERTER_LADO_DIREITO = false;
  private static final int ID_PIGEON = 22;

  // ---------------------------------------------------------------------------
  // Simulation parameters
  // ---------------------------------------------------------------------------

  private static final MomentOfInertia INERCIA_DIRECAO = KilogramSquareMeters.of(0.004);
  private static final MomentOfInertia INERCIA_TRACAO = KilogramSquareMeters.of(0.025);
  private static final Voltage ATRITO_DIRECAO = Volts.of(0.2);
  private static final Voltage ATRITO_TRACAO = Volts.of(0.2);

  // ---------------------------------------------------------------------------
  // Drivetrain and CAN bus
  // ---------------------------------------------------------------------------

  public static final SwerveDrivetrainConstants DrivetrainConstants =
      new SwerveDrivetrainConstants()
          .withPigeon2Id(ID_PIGEON)
          .withPigeon2Configs(CONFIGURACAO_PIGEON);

  public static final CANBus CAN_BUS = new CANBus(DrivetrainConstants.CANBusName);

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

  // ---------------------------------------------------------------------------
  // Module definitions
  // ---------------------------------------------------------------------------

  /** Distance of each module from the robot center along each axis. */
  private static final Distance MEIA_DISTANCIA_MODULOS =
      Meters.of(Constants.Swerve.MEIA_DISTANCIA_ENTRE_RODAS_METROS);

  /** Front-left module (steer 2, drive 1, encoder 9). */
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontLeft =
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
      FrontRight =
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
      BackLeft =
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
      BackRight =
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

  private TunerConstants() {}
}
