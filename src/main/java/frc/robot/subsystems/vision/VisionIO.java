package frc.robot.subsystems.vision;

public interface VisionIO {
    class VisionIOInputs {
        public double tx = 0.0;
        public double ty = 0.0;
        public boolean temAlvo = false;
        public int aprilTagId = -1;
    }

    default void updateInputs(VisionIOInputs inputs) {}

    default void definirPipeline(int pipeline) {}
}
