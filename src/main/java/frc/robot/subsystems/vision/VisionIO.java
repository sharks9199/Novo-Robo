package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
    @AutoLog
    class VisionIOInputs {
        public boolean temAlvo = false;
        public int aprilTagID = -1;
        public int quantidadeTagsMegaTag1 = 0;
        public int quantidadeTagsMegaTag2 = 0;
        public double distanciaMediaTagsMegaTag1 = 0.0;
        public double distanciaMediaTagsMegaTag2 = 0.0;
        public double ambiguidadeMinimaMegaTag1 = 1.0;
        public double ambiguidadeMinimaMegaTag2 = 1.0;
        public Pose2d poseMegaTag1 = new Pose2d();
        public double timestampMegaTag1 = 0.0;
        public Pose2d poseMegaTag2 = new Pose2d();
        public double timestampMegaTag2 = 0.0;
        public double tx = 0.0;
        public double ty = 0.0;
    }

    default void updateInputs(VisionIOInputs inputs) {}
    default void definirPipeline(int pipeline) {}
    default void definirOrientacao(double yawGraus, double yawRateGrausPorSegundo) {}
}
