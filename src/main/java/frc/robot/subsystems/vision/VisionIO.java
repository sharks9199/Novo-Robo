package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO {
    class VisionIOInputs {
        public double tx = 0.0;
        public double ty = 0.0;
        public boolean temAlvo = false;
        public int aprilTagId = -1;

        public Pose2d poseMegaTag1 = new Pose2d();
        public double timestampMegaTag1 = 0.0;
        public int quantidadeTagsMegaTag1 = 0;
        public double distanciaMediaTagsMegaTag1 = 0.0;
        public double ambiguidadeMinimaMegaTag1 = 1.0;

        public Pose2d poseMegaTag2 = new Pose2d();
        public double timestampMegaTag2 = 0.0;
        public int quantidadeTagsMegaTag2 = 0;
        public double distanciaMediaTagsMegaTag2 = 0.0;
    }

    default void updateInputs(VisionIOInputs inputs) {}

    default void definirPipeline(int pipeline) {}

    default void definirOrientacaoRobo(
        double yawGraus,
        double yawRateGrausPorSegundo
    ) {}
}
