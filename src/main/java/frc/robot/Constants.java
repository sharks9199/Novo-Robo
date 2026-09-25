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

        public static final double VELOCIDADE_MAXIMA_MPS = 6.5;
        public static final double ACELERACAO_MAXIMA_MPS2 = 9.0;

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

    public static final class ModoReto {
        public static final double JANELA_SNAP_ZERO_RAD =
            Units.degreesToRadians(20.0);

        // Limite de translacao quando o botao A (Modo RETO) esta ligado.
        // Com stick em 100%, o alvo de translacao e exatamente 3,0 m/s.
        public static final double VELOCIDADE_TRANSLACAO_MAXIMA_MPS = 6.0;

        // 6 m/s² faz o robo chegar aos 3 m/s rapidamente sem um degrau seco.
        public static final double ACELERACAO_TRANSLACAO_MAXIMA_MPS2 = 5.0;

        public static final double TOLERANCIA_ALINHAMENTO_MODULO_RAD =
            Units.degreesToRadians(2.0);

        // Jerk alto o suficiente para nao deixar o A "lerdo", mas ainda suave.
        public static final double JERK_TRANSLACAO_MAXIMO_MPS3 = 50.0;
        public static final double JERK_ANGULAR_MAXIMO_RAD_S3 =
            Units.degreesToRadians(6000.0);

        public static final double ACELERACAO_RODA_MAXIMA_MPS2 = 15.0;

        /**
         * Piso da cosine compensation do modo RETO.
         * 0.60 significa que a compensacao nunca reduz a velocidade da roda
         * para menos de 60% do valor que o setpoint generator ja limitou.
         * Em 3,0 m/s de comando cheio, o piso equivalente e 1,8 m/s.
         * Como e proporcional, comandos lentos continuam lentos e controlaveis.
         */
        public static final double FATOR_MINIMO_COSINE_COMPENSATION = 0.60;

        public static final double VELOCIDADE_STEER_MAXIMA_RAD_S =
            Units.degreesToRadians(900.0);

        private ModoReto() {}
    }

    public static final class SysId {
        public static final double RAMP_RATE_VOLTS_POR_SEGUNDO = 1.0;
        public static final double STEP_VOLTS = 4.0;
        public static final double TIMEOUT_SEGUNDOS = 8.0;

        private SysId() {}
    }

    public static final class Heading {
        public static final double KP = 4.0;
        public static final double KI = 0.0;
        public static final double KD = 0.15;

        public static final double VELOCIDADE_CORRECAO_MAXIMA_RAD_S =
            Units.degreesToRadians(120.0);
        public static final double ACELERACAO_CORRECAO_MAXIMA_RAD_S2 =
            Units.degreesToRadians(540.0);

        public static final double TOLERANCIA_POSICAO_RAD =
            Units.degreesToRadians(0.50);

        public static final double TOLERANCIA_VELOCIDADE_RAD_S =
            Units.degreesToRadians(4.0);

        // Entrada bruta do joystick. Pequenos desvios laterais são tratados como
        // ruído quando a intenção do piloto é andar somente para frente/trás.
        public static final double ENTRADA_LONGITUDINAL_MINIMA_RETO = 0.08;
        public static final double LATERAL_MAXIMA_ENTRAR_RETO = 0.10;
        public static final double LATERAL_MAXIMA_SAIR_RETO = 0.14;

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
