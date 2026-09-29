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
        public static final double MOMENTO_INERCIA_ESTIMADO_KG_M2 = 2.0;

        public static final double LIMITE_CORRENTE_TRACAO_AMPERES = 60.0;
        public static final double LIMITE_CORRENTE_DIRECAO_AMPERES = 40.0;

        public static final double VELOCIDADE_MAXIMA_MPS = 5;//Olha, eu (Tomboly), decidi o seguinte: velocidade / tempo = 2.5

        /** Tempo para sair do repouso e chegar na velocidade maxima (translacao). */
        public static final double TEMPO_ACELERACAO_SEGUNDOS = 1.5;
        /** Derivada: 6,5 m/s em 1,5 s = ~4,33 m/s². */
        public static final double ACELERACAO_MAXIMA_MPS2 =
            VELOCIDADE_MAXIMA_MPS / TEMPO_ACELERACAO_SEGUNDOS;

        public static final double VELOCIDADE_ANGULAR_MAXIMA_RAD_S =
            Units.degreesToRadians(720.0);
        public static final double ACELERACAO_ANGULAR_MAXIMA_RAD_S2 =
            Units.degreesToRadians(1440.0);

        public static final double DEADBAND_JOYSTICK = 0.05;
        public static final double PERIODO_CONTROLE_SEGUNDOS = 0.02;

        private Chassi() {}
    }


    public static final class ControlesDrive {
        /** Deadband radial global do stick esquerdo. */
        public static final double DEADBAND_RADIAL = 0.06;
        /** Potencia aplicada na magnitude do stick (2 = resposta quadratica). */
        public static final double EXPO_TRANSLACAO = 2.0;
        /** Eixo principal minimo para considerar snap cardinal. */
        public static final double EIXO_PRINCIPAL_MINIMO_SNAP = 0.18;
        /** Entra no snap quando o eixo secundario esta abaixo deste valor. */
        public static final double EIXO_SECUNDARIO_ENTRAR_SNAP = 0.10;
        /** Histerese: so sai do snap quando ultrapassa este valor. */
        public static final double EIXO_SECUNDARIO_SAIR_SNAP = 0.14;

        private ControlesDrive() {}
    }

    public static final class Heading {
        public static final double KP = 5;
        public static final double KI = 0.0;
        public static final double KD = 0.25;

        public static final double VELOCIDADE_CORRECAO_MAXIMA_RAD_S =
            Units.degreesToRadians(120.0);
        public static final double ACELERACAO_CORRECAO_MAXIMA_RAD_S2 =
            Units.degreesToRadians(540.0);

        public static final double TOLERANCIA_POSICAO_RAD =
            Units.degreesToRadians(0.1);

        public static final double TOLERANCIA_VELOCIDADE_RAD_S =
            Units.degreesToRadians(2.0);

        private Heading() {}
    }

    /** Ganhos e parametros fisicos usados pelo PathPlanner no autonomo. */
    public static final class Autonomo {
        // Ponto de partida para tuning: sobem se o robo fica atrasado no trajeto.
        public static final double KP_TRANSLACAO = 5.0;
        public static final double KP_ROTACAO = 5.0;

        // Coeficiente de atrito roda/carpete. Confirmar com o robo real.
        public static final double COEFICIENTE_ATRITO_RODA = 1.1;

        private Autonomo() {}
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
