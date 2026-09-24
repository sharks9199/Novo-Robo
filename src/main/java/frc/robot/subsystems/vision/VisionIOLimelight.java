package frc.robot.subsystems.vision;

import frc.robot.LimelightHelpers;

public class VisionIOLimelight implements VisionIO {
    private final String nomeLimelight;

    public VisionIOLimelight(String nomeLimelight) {
        this.nomeLimelight = nomeLimelight;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.temAlvo = LimelightHelpers.getTV(nomeLimelight);
        inputs.tx = LimelightHelpers.getTX(nomeLimelight);
        inputs.ty = LimelightHelpers.getTY(nomeLimelight);
        inputs.aprilTagId = inputs.temAlvo ? (int) LimelightHelpers.getFiducialID(nomeLimelight) : -1;
        inputs.area = LimelightHelpers.getTA(nomeLimelight);
    }

    @Override
    public void definirPipeline(int pipeline) {
        LimelightHelpers.setPipelineIndex(nomeLimelight, pipeline);
    }

    public LimelightHelpers.PoseEstimate pegarPoseMegaTag2() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(nomeLimelight);
    }
}
