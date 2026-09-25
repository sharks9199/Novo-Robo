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
        public static final int PORTA_OPERADOR = 1  ;

        private Controles() {}
    }

    public static final class Chassi {
        public static final double MASSA_COM_BATERIA_E_BUMPERS_KG = 30.0;
        public static final double DISTANCIA_ENTRE_RODAS_METROS =
            Units.inchesToMeters(21.0);

        public static final double MEIA_DISTANCIA_ENTRE_RODAS_METROS =
            DISTANCIA_ENTRE_RODAS_METROS / 2.0;

        public static final double MOMENTO_INERCIA_ESTIMADO_KG_M2 = 1.42;

        public static final double LIMITE_CORRENTE_TRACAO_AMPERES = 40.0;
        public static final double LIMITE_CORRENTE_DIRECAO_AMPERES = 40.0;

        // Modo normal continua rápido.
        public static final double VELOCIDADE_MAXIMA_MPS = 3.0;
        public static final double ACELERACAO_MAXIMA_MPS2 = 4.5;

        public static final double VELOCIDADE_ANGULAR_MAXIMA_RAD_S =
            Units.degreesToRadians(360.0);

        public static final double ACELERACAO_ANGULAR_MAXIMA_RAD_S2 =
            Units.degreesToRadians(720.0);

        public static final double DEADBAND_JOYSTICK = 0.05;
        public static final double PERIODO_CONTROLE_SEGUNDOS = 0.02;

        private Chassi() {}
    }

    public static final class ControlesDrive {
        public static final double DEADBAND_RADIAL = 0.06;

        public static final double EXPO_TRANSLACAO = 2.0;

        public static final double EIXO_PRINCIPAL_MINIMO_SNAP = 0.18;

        public static final double EIXO_SECUNDARIO_ENTRAR_SNAP = 0.10;

        public static final double EIXO_SECUNDARIO_SAIR_SNAP = 0.14;

        private ControlesDrive() {}
    }

    public static final class ModoReto {

        // Se o robô terminar um giro manual dentro desta janela,
        // ele pode voltar automaticamente para o zero do Modo RETO.
        public static final double JANELA_SNAP_ZERO_RAD =
            Units.degreesToRadians(20.0);

        /*
         * VELOCIDADE MÁXIMA DO MODO RETO
         *
         * Antes: 3.0 m/s
         * Agora: 1.8 m/s
         *
         * Essa é a principal constante para deixar o A mais lento.
         */
        public static final double VELOCIDADE_TRANSLACAO_MAXIMA_MPS = 2.2;

        /*
         * VELOCIDADE MÍNIMA PRESERVADA PELA COSINE COMPENSATION.
         *
         * Não faz o robô andar sozinho a 0.45 m/s.
         *
         * Apenas evita que um comando que já estava acima desse valor
         * seja reduzido exageradamente pela cosine compensation.
         */
        public static final double VELOCIDADE_TRANSLACAO_MINIMA_MPS = 0.4;

        /*
         * Expo maior = mais controle fino no começo do analógico.
         *
         * 1.0 = linear
         * 2.0 = bastante suave em baixa velocidade
         *
         * 1.75 dá bastante precisão sem deixar o stick morto.
         */
        public static final double EXPO_TRANSLACAO = 2.0;

        /*
         * Aceleração máxima do Modo RETO.
         *
         * Antes: 8.5 m/s²
         * Agora: 3.0 m/s²
         */
        public static final double ACELERACAO_TRANSLACAO_MAXIMA_MPS2 = 3.0;

        public static final double TOLERANCIA_ALINHAMENTO_MODULO_RAD =
            Units.degreesToRadians(2.0);

        /*
         * Jerk translacional.
         *
         * Antes: 80 m/s³
         * Agora: 20 m/s³
         *
         * Faz a aceleração entrar de forma muito mais progressiva.
         */
        public static final double JERK_TRANSLACAO_MAXIMO_MPS3 = 20.0;

        /*
         * Jerk da rotação.
         *
         * Reduzido para deixar correções e mudanças de rotação
         * menos violentas.
         */
        public static final double JERK_ANGULAR_MAXIMO_RAD_S3 =
            Units.degreesToRadians(1200.0);

        /* 
         * Limite de mudança da velocidade das rodas.
         *
         * Antes: 15 m/s²
         * Agora: 6 m/s²
         */
        public static final double ACELERACAO_RODA_MAXIMA_MPS2 = 6.0;

        /*
         * Velocidade máxima de giro dos módulos.
         *
         * Não precisa ser extremamente baixa,
         * porque queremos que os módulos alinhem rapidamente.
         */
        public static final double VELOCIDADE_STEER_MAXIMA_RAD_S =
            Units.degreesToRadians(720.0);

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

        /*
         * Eu NÃO reduziria demais isso.
         *
         * A velocidade translacional pode ser lenta,
         * mas o robô ainda precisa conseguir corrigir heading.
         */
        public static final double VELOCIDADE_CORRECAO_MAXIMA_RAD_S =
            Units.degreesToRadians(90.0);

        public static final double ACELERACAO_CORRECAO_MAXIMA_RAD_S2 =
            Units.degreesToRadians(360.0);

        public static final double TOLERANCIA_POSICAO_RAD =
            Units.degreesToRadians(0.50);

        public static final double TOLERANCIA_VELOCIDADE_RAD_S =
            Units.degreesToRadians(4.0);

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