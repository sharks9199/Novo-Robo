package frc.robot.subsystems.vision;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;

public class VisionIOLimelight implements VisionIO {
    private final String nomeLimelight;

    public VisionIOLimelight(String nomeLimelight) {
        this.nomeLimelight = nomeLimelight;
    }

    @Override
    public void definirOrientacaoRobo(
        double yawGraus,
        double yawRateGrausPorSegundo
    ) {
        LimelightHelpers.SetRobotOrientation(
            nomeLimelight,
            yawGraus,
            yawRateGrausPorSegundo,
            0.0,
            0.0,
            0.0,
            0.0
        );
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.temAlvo = LimelightHelpers.getTV(nomeLimelight);
        inputs.tx = LimelightHelpers.getTX(nomeLimelight);
        inputs.ty = LimelightHelpers.getTY(nomeLimelight);
        inputs.aprilTagId = inputs.temAlvo
            ? (int) LimelightHelpers.getFiducialID(nomeLimelight)
            : -1;

        PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(nomeLimelight);
        inputs.poseMegaTag1 = mt1.pose;
        inputs.timestampMegaTag1 = mt1.timestampSeconds;
        inputs.quantidadeTagsMegaTag1 = mt1.tagCount;
        inputs.distanciaMediaTagsMegaTag1 = mt1.avgTagDist;
        inputs.ambiguidadeMinimaMegaTag1 = menorAmbiguidade(mt1.rawFiducials);

        PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(nomeLimelight);
        inputs.poseMegaTag2 = mt2.pose;
        inputs.timestampMegaTag2 = mt2.timestampSeconds;
        inputs.quantidadeTagsMegaTag2 = mt2.tagCount;
        inputs.distanciaMediaTagsMegaTag2 = mt2.avgTagDist;
    }

    @Override
    public void definirPipeline(int pipeline) {
        LimelightHelpers.setPipelineIndex(nomeLimelight, pipeline);
    }

    private static double menorAmbiguidade(RawFiducial[] fiduciais) {
        if (fiduciais == null || fiduciais.length == 0) {
            return 1.0;
        }

        double menor = Double.POSITIVE_INFINITY;
        for (RawFiducial fiducial : fiduciais) {
            menor = Math.min(menor, fiducial.ambiguity);
        }
        return menor;
    }
}
