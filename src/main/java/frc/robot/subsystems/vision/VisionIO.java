package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;

public interface VisionIO {
    class VisionIOInputs {
        public Pose3d robotPose = new Pose3d();
        public boolean temAlvo = false;
        public double timestamp = 0.0;
        public int tagCount = 0;
        public int aprilTagId = -1;

        public double ta = 0.0;
        public double tx = 0.0;
        public double ty = 0.0;
        
        public double pipelineIndex = 0.0;
    }

    default void updateInputs(VisionIOInputs inputs) {}

    default void definirPipeline(int pipeline) {}
}
