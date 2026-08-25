package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;

public final class OffsetsConstants {

    public static final class Driver {
        public static final double FRENTE_GRAUS = -90.0; //FINALMENTE CONSEGUIII!!!

        private Driver() {}
    }
    public static final class Swerve {

        /* CONSTANTES CERTAS QUE DÃO QUASE CERTAS
        public static final Angle FRENTE_ESQUERDA = Rotations.of(-0.1435546875); //silva
        public static final Angle FRENTE_DIREITA = Rotations.of(0.3017578125); //silva
        public static final Angle TRAS_ESQUERDA = Rotations.of(0.208251953125); //silva
        public static final Angle TRAS_DIREITA = Rotations.of(-0.353759765625); //silva

        */

        //CONSTANTES NOVAS AJUSTADAS A
        public static final Angle FRENTE_ESQUERDA = Rotations.of(-0.13671875); //silva
        public static final Angle FRENTE_DIREITA = Rotations.of(0.29541015625); //silva
        public static final Angle TRAS_ESQUERDA = Rotations.of(0.199462890625); //silva
        public static final Angle TRAS_DIREITA = Rotations.of(-0.366455078125); //silva


        private Swerve() {}
    }

    public static final class Limelight {
        public static final double TX_ALVO_GRAUS = 0.0;

        private Limelight() {}
    }

    public static final class Pigeon {
        public static final Angle YAW = Degrees.of(0.0);
        public static final Angle PITCH = Degrees.of(0.0);
        public static final Angle ROLL = Degrees.of(0.0);

        private Pigeon() {}
    }

    private OffsetsConstants() {}
}