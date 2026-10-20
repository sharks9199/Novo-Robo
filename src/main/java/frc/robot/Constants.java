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

public final class Constants {
    public static final Mode MODO_SIMULACAO = Mode.SIM;
    public static final Mode MODO_ATUAL =
        RobotBase.isReal() ? Mode.REAL : MODO_SIMULACAO;

    public enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    public static final class Controles {
        public static final int PORTA_DRIVER = 0;
        public static final int PORTA_OPERADOR = 1;

        private Controles() {}
    }

    public static final class Swerve {
        public static final double MASSA_COM_BATERIA_E_BUMPERS_KG = 34.3;

        public static final double DISTANCIA_ENTRE_RODAS_METROS = Units.inchesToMeters(21.0);

        public static final double MEIA_DISTANCIA_ENTRE_RODAS_METROS = DISTANCIA_ENTRE_RODAS_METROS / 2.0;

        public static final double MOMENTO_INERCIA_ESTIMADO_KG_M2 = 2.491;

        public static final double LIMITE_CORRENTE_TRACAO_AMPERES = 60.0;
        public static final double LIMITE_CORRENTE_DIRECAO_AMPERES = 40.0;

        public static final double VELOCIDADE_MAXIMA_MPS = 4.02;
        public static final double ACELERACAO_MAXIMA_MPS2 = 7.5;

        public static final double VELOCIDADE_ANGULAR_MAXIMA_RAD_S = Units.degreesToRadians(540.0);
        public static final double ACELERACAO_ANGULAR_MAXIMA_RAD_S2 = Units.degreesToRadians(720.0);



        public static final double VELOCIDADE_MAXIMA_MPS_LENTO = 2.0;
        public static final double ACELERACAO_MAXIMA_MPS_LENTO = 1.0;



        public static final double DEADBAND_JOYSTICK = 0.03;
        public static final double PERIODO_CONTROLE_SEGUNDOS = 0.02;

        private static final Slot0Configs GANHOS_DIRECAO =
            new Slot0Configs()
                .withKP(25.0)
                .withKI(0.0)
                .withKD(2.5)
                .withKS(0.0)
                .withKV(0.12)
                .withKA(0.0)
                .withStaticFeedforwardSign(
                    StaticFeedforwardSignValue.UseClosedLoopSign
                );

        private static final Slot0Configs GANHOS_TRACAO =
            new Slot0Configs()
                .withKP(0.2)
                .withKI(0.0)
                .withKD(0.08)
                .withKS(0.24)
                .withKV(0.12);


/*        private static final Slot0Configs GANHOS_DIRECAO =
            new Slot0Configs()
                .withKP(25.0)
                .withKI(0.0)
                .withKD(2.5)
                .withKS(0.0)
                .withKV(0.12)
                .withKA(0.0)
                .withStaticFeedforwardSign(
                    StaticFeedforwardSignValue.UseClosedLoopSign
                );

        private static final Slot0Configs GANHOS_TRACAO =
            new Slot0Configs()
                .withKP(0.2)
                .withKI(0.0)
                .withKD(0.08)
                .withKS(0.24)
                .withKV(0.12); */
        private static final ClosedLoopOutputType SAIDA_DIRECAO = ClosedLoopOutputType.Voltage;

        private static final ClosedLoopOutputType SAIDA_TRACAO = ClosedLoopOutputType.Voltage;

        private static final DriveMotorArrangement MOTOR_TRACAO = DriveMotorArrangement.TalonFX_Integrated;

        private static final SteerMotorArrangement MOTOR_DIRECAO = SteerMotorArrangement.TalonFX_Integrated;

        private static final SteerFeedbackType SENSOR_DIRECAO = SteerFeedbackType.FusedCANcoder;

        private static final Current CORRENTE_DE_ESCORREGAMENTO = Amps.of(LIMITE_CORRENTE_TRACAO_AMPERES);

        private static final TalonFXConfiguration CONFIGURACAO_TRACAO = new TalonFXConfiguration();

        private static final TalonFXConfiguration CONFIGURACAO_DIRECAO =
            new TalonFXConfiguration()
                .withCurrentLimits(
                    new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(
                            Amps.of(LIMITE_CORRENTE_DIRECAO_AMPERES)
                        )
                        .withStatorCurrentLimitEnable(true)
                );

        private static final CANcoderConfiguration CONFIGURACAO_ENCODER =
            new CANcoderConfiguration();

        public static final Pigeon2Configuration CONFIGURACAO_PIGEON =
            new Pigeon2Configuration()
                .withMountPose(
                    new MountPoseConfigs()
                        .withMountPoseYaw(OffsetsConstants.Pigeon.YAW)
                        .withMountPosePitch(OffsetsConstants.Pigeon.PITCH)
                        .withMountPoseRoll(OffsetsConstants.Pigeon.ROLL)
                );

        public static final LinearVelocity VELOCIDADE_FISICA_A_12V =
            MetersPerSecond.of(4.20);

        private static final double RELACAO_ACOPLAMENTO = 25.0 / 7.0;
        private static final double REDUCAO_TRACAO = 6.75;
        private static final double REDUCAO_DIRECAO = 150.0 / 7.0;

        private static final Distance RAIO_RODA =
            Meters.of(0.0964 / 2); //diâmetro: 96.8mm = 48,4mm

        private static final boolean INVERTER_LADO_ESQUERDO = true;
        private static final boolean INVERTER_LADO_DIREITO = false;

        private static final int ID_PIGEON = 22;

        private static final MomentOfInertia INERCIA_DIRECAO = KilogramSquareMeters.of(0.004);

        private static final MomentOfInertia INERCIA_TRACAO = KilogramSquareMeters.of(0.020);

        private static final Voltage ATRITO_DIRECAO = Volts.of(0.2);

        private static final Voltage ATRITO_TRACAO = Volts.of(0.25);

        public static final SwerveDrivetrainConstants DRIVETRAIN = new SwerveDrivetrainConstants()
                .withPigeon2Id(ID_PIGEON)
                .withPigeon2Configs(CONFIGURACAO_PIGEON);

        public static final CANBus CAN_BUS = new CANBus(DRIVETRAIN.CANBusName);

        private static final SwerveModuleConstantsFactory<
            TalonFXConfiguration,
            TalonFXConfiguration,
            CANcoderConfiguration
        > FABRICA_MODULOS =
            new SwerveModuleConstantsFactory<
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

        private static final Distance MEIA_DISTANCIA_MODULOS =
            Meters.of(MEIA_DISTANCIA_ENTRE_RODAS_METROS);

        public static final SwerveModuleConstants<
            TalonFXConfiguration,
            TalonFXConfiguration,
            CANcoderConfiguration
        > FRONT_LEFT = criarModulo(
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
        > FRONT_RIGHT = criarModulo(
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
        > BACK_LEFT = criarModulo(
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
        > BACK_RIGHT = criarModulo(
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

        private Swerve() {}
    }

    public static final class ControlesDrive {
        public static final double DEADBAND_RADIAL = 0.07;
        public static final double EXPO_TRANSLACAO = 3;
        public static final double EIXO_PRINCIPAL_MINIMO_SNAP = 0.18;
        public static final double EIXO_SECUNDARIO_ENTRAR_SNAP = 0.11;
        public static final double EIXO_SECUNDARIO_SAIR_SNAP = 0.14;

        private ControlesDrive() {}
    }

    public static final class Heading {
        public static final double KP = 30.0;
        public static final double KI = 0.0;
        public static final double KD = 10.0;

        public static final double VELOCIDADE_CORRECAO_MAXIMA_RAD_S = Units.degreesToRadians(120.0);
        public static final double ACELERACAO_CORRECAO_MAXIMA_RAD_S2 = Units.degreesToRadians(540.0);

        public static final double TOLERANCIA_POSICAO_RAD = Units.degreesToRadians(0.1);
        public static final double TOLERANCIA_VELOCIDADE_RAD_S = Units.degreesToRadians(2.0);

        private Heading() {}
    }

    public static final class Autonomo {
        public static final double KP_TRANSLACAO = 4.0;
        public static final double KP_ROTACAO = 4.0;
        public static final double COEFICIENTE_ATRITO_RODA = 1.15;

        private Autonomo() {}
    }

    public static final class Vision {
        public static final String NOME_LIMELIGHT = "limelight-sharks";

        public static final double DISTANCIA_MAXIMA_VISAO_METROS = 6.0;
        public static final double VELOCIDADE_ANGULAR_MAXIMA_PARA_VISAO_RAD_S = Units.degreesToRadians(540.0);
        public static final double DISTANCIA_MAXIMA_TAG_UNICA_HEADING_METROS = 2.5;
        public static final double AMBIGUIDADE_MAXIMA_TAG_UNICA_HEADING = 0.15;

        public static final double MT1_DESVIO_XY_MULTIPLAS_TAGS_METROS = 0.45;
        public static final double MT1_DESVIO_XY_TAG_UNICA_METROS = 0.80;
        public static final double MT1_DESVIO_THETA_MULTIPLAS_TAGS_RAD = Units.degreesToRadians(4.0);
        public static final double MT1_DESVIO_THETA_TAG_UNICA_RAD = Units.degreesToRadians(10.0);

        public static final double MT2_DESVIO_XY_MULTIPLAS_TAGS_METROS = 0.30;
        public static final double MT2_DESVIO_XY_TAG_UNICA_METROS = 0.70;
        public static final double MT2_DESVIO_THETA_RAD = 1_000_000.0;

        private Vision() {}
    }

    private Constants() {}
}