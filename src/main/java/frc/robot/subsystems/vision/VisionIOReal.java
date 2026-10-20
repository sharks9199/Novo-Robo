package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

/** Limelight NetworkTables adapter. Requires Limelight OS 2026+ for botpose_orb_wpiblue. */
public class VisionIOReal implements VisionIO {
    private final NetworkTable table = NetworkTableInstance.getDefault()
            .getTable(Constants.Vision.NOME_LIMELIGHT);
    private final NetworkTableEntry orientation = table.getEntry("robot_orientation_set");

    @Override
    public void definirPipeline(int pipeline) {
        table.getEntry("pipeline").setDouble(pipeline);
    }

    @Override
    public void definirOrientacao(double yawGraus, double yawRateGrausPorSegundo) {
        // Limelight MegaTag2 input: yaw, yaw rate, pitch, pitch rate, roll, roll rate.
        orientation.setDoubleArray(new double[] {yawGraus, yawRateGrausPorSegundo, 0, 0, 0, 0});
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.tx = table.getEntry("tx").getDouble(0.0);
        inputs.ty = table.getEntry("ty").getDouble(0.0);
        inputs.temAlvo = table.getEntry("tv").getDouble(0.0) >= 1.0;

        double[] mt1 = table.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);
        double[] mt2 = table.getEntry("botpose_orb_wpiblue").getDoubleArray(new double[0]);
        preencherPose(mt1, false, inputs);
        preencherPose(mt2, true, inputs);

        double[] fiducials = table.getEntry("rawfiducials").getDoubleArray(new double[0]);
        inputs.aprilTagID = -1;
        double menorAmbiguidade = 1.0;
        for (int i = 0; i + 6 < fiducials.length; i += 7) {
            int id = (int) Math.round(fiducials[i]);
            double ambiguidade = fiducials[i + 6];
            if (inputs.aprilTagID < 0) inputs.aprilTagID = id;
            if (ambiguidade >= 0 && ambiguidade < menorAmbiguidade) {
                menorAmbiguidade = ambiguidade;
            }
        }
        inputs.ambiguidadeMinimaMegaTag1 = menorAmbiguidade;
        inputs.ambiguidadeMinimaMegaTag2 = menorAmbiguidade;
        inputs.temAlvo = inputs.temAlvo || fiducials.length >= 7;
    }

    private void preencherPose(double[] dados, boolean megaTag2, VisionIOInputs inputs) {
        // Limelight pose array: x,y,z,roll,pitch,yaw,latency(ms),tagCount,tagSpan,avgDist,avgArea,...
        if (dados.length < 11) {
            if (megaTag2) {
                inputs.quantidadeTagsMegaTag2 = 0;
                inputs.distanciaMediaTagsMegaTag2 = 0.0;
            } else {
                inputs.quantidadeTagsMegaTag1 = 0;
                inputs.distanciaMediaTagsMegaTag1 = 0.0;
            }
            return;
        }
        Pose2d pose = new Pose2d(dados[0], dados[1], Rotation2d.fromDegrees(dados[5]));
        double timestamp = Timer.getFPGATimestamp() - Math.max(0.0, dados[6]) / 1000.0;
        int count = (int) Math.round(dados[7]);
        double distance = dados[9];
        if (megaTag2) {
            inputs.poseMegaTag2 = pose;
            inputs.timestampMegaTag2 = timestamp;
            inputs.quantidadeTagsMegaTag2 = count;
            inputs.distanciaMediaTagsMegaTag2 = distance;
        } else {
            inputs.poseMegaTag1 = pose;
            inputs.timestampMegaTag1 = timestamp;
            inputs.quantidadeTagsMegaTag1 = count;
            inputs.distanciaMediaTagsMegaTag1 = distance;
        }
    }
}
