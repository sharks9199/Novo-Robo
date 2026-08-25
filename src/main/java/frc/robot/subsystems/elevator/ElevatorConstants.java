package frc.robot.subsystems.elevator;

public final class ElevatorConstants {
    public static final class Hardware {
        public static final int ID_MOTOR_ESQUERDO = 13;
        public static final int ID_MOTOR_DIREITO = 14;
        public static final int PORTA_ENCODER_ABSOLUTO = 9;
        public static final int LIMITE_CORRENTE_AMPERES = 40;

        public static final boolean INVERTER_MOTOR_ESQUERDO = false;
        public static final boolean INVERTER_MOTOR_DIREITO = false;
        public static final boolean INVERTER_ENCODER_ESQUERDO = true;
        public static final boolean INVERTER_ENCODER_DIREITO = false;

        private Hardware() {}
    }

    public static final class EncoderAbsoluto {
        public static final boolean INVERTIDO = false;
        public static final double OFFSET_ROTACOES = 0.0;
        public static final double LEITURA_MINIMA_VALIDA = 0.001;
        public static final double LEITURA_MAXIMA_VALIDA = 0.999;
        public static final int FREQUENCIA_MINIMA_HZ = 20;

        private EncoderAbsoluto() {}
    }

    public static final class Altura {
        public static final double MINIMA_CM = 0.0;
        public static final double MAXIMA_CM = 100.0;
        public static final double ROTACOES_NA_ALTURA_MINIMA = 0.0;
        public static final double ROTACOES_NA_ALTURA_MAXIMA = 80.0;

        public static final double KP = 0.65;
        public static final double KI = 0.0;
        public static final double KD = 0.03;
        public static final double FF_VOLTS = 0.0;

        public static final double TOLERANCIA_ROTACOES = 0.35;
        public static final double TOLERANCIA_VELOCIDADE = 1.0;
        public static final double TENSAO_MAXIMA_AUTOMATICA = 8.0;

        public static final double TENSAO_MANUAL_SUBIDA = 5.0;
        public static final double TENSAO_MANUAL_DESCIDA = 3.5;

        public static final double VELOCIDADE_MAXIMA_NORMAL = 35.0;
        public static final double ACELERACAO_MAXIMA_NORMAL = 70.0;
        public static final double VELOCIDADE_MAXIMA_INTERMEDIARIA = 16.0;
        public static final double ACELERACAO_MAXIMA_INTERMEDIARIA = 28.0;
        public static final double VELOCIDADE_MAXIMA_DESCIDA_FINAL = 8.0;
        public static final double ACELERACAO_MAXIMA_DESCIDA_FINAL = 14.0;

        public static final double ALTURA_INTERMEDIARIA_CM = 18.0;
        public static final double DIFERENCA_MAXIMA_ENTRE_ENCODERS_ROTACOES = 1.5;
        public static final double INTENSIDADE_FILTRO = 0.35;

        private Altura() {}
    }

    public static final class Giro {
        public static final boolean PERMITE_VOLTA_COMPLETA = false;
        public static final double ANGULO_MINIMO_GRAUS = 5.0;
        public static final double ANGULO_MAXIMO_GRAUS = 355.0;
        public static final double TOLERANCIA_GRAUS = 2.0;
        public static final double SENTIDO_TENSAO = 1.0;

        public static final double KP = 0.09;
        public static final double KI = 0.0;
        public static final double KD = 0.002;
        public static final double TENSAO_MAXIMA_AUTOMATICA = 4.0;
        public static final double TENSAO_MANUAL = 3.0;
        public static final double FATOR_GIRO_MOTOR_ESQUERDO = 1.0;
        public static final double FATOR_GIRO_MOTOR_DIREITO = 0.92;

        private Giro() {}
    }

    public static final class Rampa {
        public static final double ALTURA_ACELERACAO_VOLTS_S = 8.0;
        public static final double ALTURA_DESACELERACAO_VOLTS_S = 12.0;
        public static final double GIRO_ACELERACAO_VOLTS_S = 7.0;
        public static final double GIRO_DESACELERACAO_VOLTS_S = 10.0;

        private Rampa() {}
    }

    public static final class Presets {
        public static final double H0_ALTURA_CM = 0.0;
        public static final double H1_ALTURA_CM = 30.0;
        public static final double H2_ALTURA_CM = 60.0;
        public static final double H3_ALTURA_CM = 90.0;

        public static final double H0_ANGULO_GRAUS = 5.0;
        public static final double H1_ANGULO_GRAUS = 45.0;
        public static final double H2_ANGULO_GRAUS = 90.0;
        public static final double H3_ANGULO_GRAUS = 135.0;

        private Presets() {}
    }

    public static final class Controle {
        public static final double LIMITE_GATILHO_DIREITO = 0.5;
        public static final double PERIODO_SEGUNDOS = 0.02;
        public static final double TENSAO_MAXIMA_MOTORES = 12.0;

        private Controle() {}
    }

    private ElevatorConstants() {}
}
