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
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.OffsetsConstants;

/**
 * Configuração física dos quatro módulos gerada a partir do Phoenix Tuner X.
 *
 * <p>Os limites de operação do robô ficam em {@link Constants.Chassi}. Esta classe mantém
 * relações mecânicas, IDs CAN, offsets e ganhos que precisam corresponder ao hardware real.
 */
public final class TunerConstants {
    private static final Slot0Configs GANHOS_DIRECAO = new Slot0Configs()
        .withKP(25.0)
        .withKI(0.0)
        .withKD(2.5)
        .withKS(0.0)
        .withKV(0.12)
        .withKA(0.0)
        .withStaticFeedforwardSign(
            StaticFeedforwardSignValue.UseClosedLoopSign
        );

    private static final Slot0Configs GANHOS_TRACAO = new Slot0Configs()
        .withKP(0.2)
        .withKI(0.0)
        .withKD(0.08)
        .withKS(0.24)
        .withKV(0.12);

    private static final ClosedLoopOutputType SAIDA_DIRECAO =
        ClosedLoopOutputType.Voltage;
    private static final ClosedLoopOutputType SAIDA_TRACAO =
        ClosedLoopOutputType.Voltage;

    private static final DriveMotorArrangement MOTOR_TRACAO =
        DriveMotorArrangement.TalonFX_Integrated;
    private static final SteerMotorArrangement MOTOR_DIRECAO =
        SteerMotorArrangement.TalonFX_Integrated;
    private static final SteerFeedbackType SENSOR_DIRECAO =
        SteerFeedbackType.FusedCANcoder;

    private static final Current CORRENTE_DE_ESCORREGAMENTO = Amps.of(
        Constants.Chassi.LIMITE_CORRENTE_TRACAO_AMPERES
    );

    private static final TalonFXConfiguration CONFIGURACAO_TRACAO =
        new TalonFXConfiguration();

    private static final TalonFXConfiguration CONFIGURACAO_DIRECAO =
        new TalonFXConfiguration().withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(
                    Amps.of(Constants.Chassi.LIMITE_CORRENTE_DIRECAO_AMPERES)
                )
                .withStatorCurrentLimitEnable(true)
        );

    private static final CANcoderConfiguration CONFIGURACAO_ENCODER =
        new CANcoderConfiguration();
    public static final Pigeon2Configuration CONFIGURACAO_PIGEON = new Pigeon2Configuration().withMountPose(new MountPoseConfigs().withMountPoseYaw(OffsetsConstants.Pigeon.YAW).withMountPosePitch(OffsetsConstants.Pigeon.PITCH).withMountPoseRoll(OffsetsConstants.Pigeon.ROLL));

    /** Velocidade física aproximada a 12 V, usada pelo modelo do módulo. */
    public static final LinearVelocity VELOCIDADE_FISICA_A_12V =
        MetersPerSecond.of(5);

    private static final double RELACAO_ACOPLAMENTO = 25.0 / 7.0;
    private static final double REDUCAO_TRACAO = 6.75;
    private static final double REDUCAO_DIRECAO = 150.0 / 7.0;
    private static final Distance RAIO_RODA = Meters.of(0.0508);

    private static final boolean INVERTER_LADO_ESQUERDO = true;
    private static final boolean INVERTER_LADO_DIREITO = false;
    private static final int ID_PIGEON = 22;

    private static final MomentOfInertia INERCIA_DIRECAO =
        KilogramSquareMeters.of(0.004);
    private static final MomentOfInertia INERCIA_TRACAO =
        KilogramSquareMeters.of(0.025);
    private static final Voltage ATRITO_DIRECAO = Volts.of(0.2);
    private static final Voltage ATRITO_TRACAO = Volts.of(0.2);

    public static final SwerveDrivetrainConstants DrivetrainConstants =
        new SwerveDrivetrainConstants()
            .withPigeon2Id(ID_PIGEON)
            .withPigeon2Configs(CONFIGURACAO_PIGEON);

    public static final CANBus CAN_BUS = new CANBus(
        DrivetrainConstants.CANBusName
    );

    private static final SwerveModuleConstantsFactory<
        TalonFXConfiguration,
        TalonFXConfiguration,
        CANcoderConfiguration
    > FABRICA_MODULOS = new SwerveModuleConstantsFactory<
        TalonFXConfiguration,
        TalonFXConfiguration,
        CANcoderConfiguration
    >()
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

    private static final Distance MEIA_DISTANCIA_MODULOS = Meters.of(
        Constants.Chassi.MEIA_DISTANCIA_ENTRE_RODAS_METROS
    );

    public static final SwerveModuleConstants<
        TalonFXConfiguration,
        TalonFXConfiguration,
        CANcoderConfiguration
    > FrontLeft = criarModulo(
        2,
        1,
        9,
        OffsetsConstants.Swerve.FRENTE_ESQUERDA,
        MEIA_DISTANCIA_MODULOS,
        MEIA_DISTANCIA_MODULOS,
        INVERTER_LADO_ESQUERDO,
        true,
        false
    );

    public static final SwerveModuleConstants<
        TalonFXConfiguration,
        TalonFXConfiguration,
        CANcoderConfiguration
    > FrontRight = criarModulo(
        4,
        3,
        10,
        OffsetsConstants.Swerve.FRENTE_DIREITA,
        MEIA_DISTANCIA_MODULOS,
        MEIA_DISTANCIA_MODULOS.unaryMinus(),
        INVERTER_LADO_DIREITO,
        true,
        false
    );

    public static final SwerveModuleConstants<
        TalonFXConfiguration,
        TalonFXConfiguration,
        CANcoderConfiguration
    > BackLeft = criarModulo(
        6,
        5,
        11,
        OffsetsConstants.Swerve.TRAS_ESQUERDA,
        MEIA_DISTANCIA_MODULOS.unaryMinus(),
        MEIA_DISTANCIA_MODULOS,
        INVERTER_LADO_ESQUERDO,
        true,
        false
    );

    public static final SwerveModuleConstants<
        TalonFXConfiguration,
        TalonFXConfiguration,
        CANcoderConfiguration
    > BackRight = criarModulo(
        8,
        7,
        12,
        OffsetsConstants.Swerve.TRAS_DIREITA,
        MEIA_DISTANCIA_MODULOS.unaryMinus(),
        MEIA_DISTANCIA_MODULOS.unaryMinus(),
        INVERTER_LADO_DIREITO,
        true,
        false
    );

    private static SwerveModuleConstants<
        TalonFXConfiguration,
        TalonFXConfiguration,
        CANcoderConfiguration
    > criarModulo(
        int idDirecao,
        int idTracao,
        int idEncoder,
        Angle offsetEncoder,
        Distance posicaoX,
        Distance posicaoY,
        boolean inverterTracao,
        boolean inverterDirecao,
        boolean inverterEncoder
    ) {
        return FABRICA_MODULOS.createModuleConstants(
            idDirecao,
            idTracao,
            idEncoder,
            offsetEncoder,
            posicaoX,
            posicaoY,
            inverterTracao,
            inverterDirecao,
            inverterEncoder
        );
    }

    private TunerConstants() {}
}