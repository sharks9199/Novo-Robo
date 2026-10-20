package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO {
    //ESTOU PROGRAMANDO AINDA. 
    //Eu estava usando a programação do Silva, mas decidi fazer a minha própria.
    public static class VisionIOInputs{
        public double tx = 0.0;
        public double ty = 0.0;
        public boolean temAlvo = false;
        public int aprilTagID = -1; //-1 -> significa q não tem

        public Pose2d poseMegaTag1 = new Pose2d();
        public double timestampMegaTag1 = 0.0;
        public int quantidadeTagsMegaTag1 = 0;
        public double distanciaMediaTagsMegaTag1 = 0.0;
        public double ambiguidadeMinimaMegaTag1 = 1.0;

        public Pose2d poseMegaTag2 = new Pose2d();
        public double timestampMegaTag2 = 0.0;
        public int quantidadeTagsMegaTag2 = 0;
        public double distanciaMediaTagsMegaTag2 = 0.0;
        public double ambiguidadeMinimaMegaTag2 = 1.0;

    }

    default void updateInputs(VisionIOInputs inputs){}

    default void definirPipeline(int pipeline){}

    default void definiOrientacao(double yawGraus, double yawRateGrausPorSegundo){}

}
