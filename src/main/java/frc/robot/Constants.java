package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
    public static final Mode MODO_SIMULACAO = Mode.SIM;
    public static final Mode MODO_ATUAL = RobotBase.isReal() ? Mode.REAL : MODO_SIMULACAO;

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

    public static final class Chassi {
        public static final double MASSA_COM_BATERIA_E_BUMPERS_KG = 30.0;
        public static final double DISTANCIA_ENTRE_RODAS_METROS = Units.inchesToMeters(21.0);
        public static final double MEIA_DISTANCIA_ENTRE_RODAS_METROS =
            DISTANCIA_ENTRE_RODAS_METROS / 2.0;
        public static final double MOMENTO_INERCIA_ESTIMADO_KG_M2 = 1.42;

        public static final double LIMITE_CORRENTE_TRACAO_AMPERES = 40.0;
        public static final double LIMITE_CORRENTE_DIRECAO_AMPERES = 40.0;

        public static final double VELOCIDADE_MAXIMA_MPS = 5.0;
        public static final double ACELERACAO_MAXIMA_MPS2 = 8.5;

        public static final double VELOCIDADE_ANGULAR_MAXIMA_RAD_S =
            Units.degreesToRadians(720.0);
        public static final double ACELERACAO_ANGULAR_MAXIMA_RAD_S2 =
            Units.degreesToRadians(1440.0);

        public static final double DEADBAND_JOYSTICK = 0.05;
        public static final double PERIODO_CONTROLE_SEGUNDOS = 0.02;

        private Chassi() {}
    }

    public static final class Heading {
        public static final double KP = 5.0;
        public static final double KI = 0.0;
        public static final double KD = 0.15;

        public static final double VELOCIDADE_CORRECAO_MAXIMA_RAD_S =
            Units.degreesToRadians(180.0);

        public static final double TOLERANCIA_POSICAO_RAD =
            Units.degreesToRadians(0.75);

        public static final double TOLERANCIA_VELOCIDADE_RAD_S =
            Units.degreesToRadians(5.0);

        private Heading() {}
    }

    public static final class Vision {
        public static final String NOME_LIMELIGHT = "limelight-sharks";

        public static final double DISTANCIA_MAXIMA_VISAO_METROS = 6.0;
        public static final double VELOCIDADE_ANGULAR_MAXIMA_PARA_VISAO_RAD_S =
            Units.degreesToRadians(540.0);

        public static final double DISTANCIA_MAXIMA_TAG_UNICA_HEADING_METROS = 2.5;
        public static final double AMBIGUIDADE_MAXIMA_TAG_UNICA_HEADING = 0.15;

        public static final double MT1_DESVIO_XY_MULTIPLAS_TAGS_METROS = 0.45;
        public static final double MT1_DESVIO_XY_TAG_UNICA_METROS = 0.80;
        public static final double MT1_DESVIO_THETA_MULTIPLAS_TAGS_RAD =
            Units.degreesToRadians(4.0);
        public static final double MT1_DESVIO_THETA_TAG_UNICA_RAD =
            Units.degreesToRadians(10.0);

        public static final double MT2_DESVIO_XY_MULTIPLAS_TAGS_METROS = 0.30;
        public static final double MT2_DESVIO_XY_TAG_UNICA_METROS = 0.70;
        public static final double MT2_DESVIO_THETA_RAD = 1_000_000.0;

        private Vision() {}
    }

    private Constants() {}
}
