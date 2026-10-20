package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.Drive;

import frc.robot.subsystems.vision.VisionIOInputsAutoLogged;

/** Reads AprilTag poses and fuses validated vision measurements into Drive odometry. */
public class Vision extends SubsystemBase {
    private final Drive drive;
    private final VisionIO io;
    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
    private double ultimoTimestampAceito = -1.0;

    public Vision(Drive drive, VisionIO io) {
        this.drive = drive;
        this.io = io;
        io.definirPipeline(Constants.Vision.PIPELINE_APRILTAG);
    }

    @Override
    public void periodic() {
        io.definirOrientacao(drive.getGyroRotation().getDegrees(),
                Math.toDegrees(drive.getYawVelocityRadPerSec()));
        io.updateInputs(inputs);
        Logger.processInputs("Vision/Limelight", inputs);

        Logger.recordOutput("Vision/AprilTag/Visible", inputs.temAlvo);
        Logger.recordOutput("Vision/AprilTag/ID", inputs.aprilTagID);
        Logger.recordOutput("Vision/MegaTag1/TagCount", inputs.quantidadeTagsMegaTag1);
        Logger.recordOutput("Vision/MegaTag2/TagCount", inputs.quantidadeTagsMegaTag2);
        Logger.recordOutput("Vision/MegaTag2/DistanceMeters", inputs.distanciaMediaTagsMegaTag2);

        // Prefer MegaTag2; it uses the Pigeon orientation supplied above and does not
        // estimate heading from the tag geometry. Never fuse the same camera frame twice.
        if (medicaoValida(inputs.poseMegaTag2, inputs.timestampMegaTag2,
                inputs.quantidadeTagsMegaTag2, inputs.distanciaMediaTagsMegaTag2)) {
            double distancia = inputs.distanciaMediaTagsMegaTag2;
            double sigmaXY = Constants.Vision.DESVIO_MT2_XY_BASE_METROS
                    + Constants.Vision.DESVIO_MT2_XY_POR_METRO * distancia;
            drive.adicionarMedicaoVisao(inputs.poseMegaTag2, inputs.timestampMegaTag2,
                    sigmaXY, sigmaXY, Constants.Vision.MT2_DESVIO_THETA_RAD);
            ultimoTimestampAceito = inputs.timestampMegaTag2;
            Logger.recordOutput("Vision/Accepted", true);
            Logger.recordOutput("Vision/AcceptedSource", "MegaTag2");
            return;
        }

        // Fallback to MegaTag1 only if MT2 is unavailable/invalid. Use heading only
        // with multiple tags; a single tag's heading is too ambiguous for reliable fusion.
        if (medicaoValida(inputs.poseMegaTag1, inputs.timestampMegaTag1,
                inputs.quantidadeTagsMegaTag1, inputs.distanciaMediaTagsMegaTag1)) {
            int tags = inputs.quantidadeTagsMegaTag1;
            double distancia = inputs.distanciaMediaTagsMegaTag1;
            double sigmaXY = Constants.Vision.DESVIO_MT1_XY_BASE_METROS
                    + Constants.Vision.DESVIO_MT1_XY_POR_METRO * distancia;
            double sigmaTheta = tags >= 2
                    ? Constants.Vision.MT1_DESVIO_THETA_MULTIPLAS_TAGS_RAD
                    : Constants.Vision.MT2_DESVIO_THETA_RAD;
            drive.adicionarMedicaoVisao(inputs.poseMegaTag1, inputs.timestampMegaTag1,
                    sigmaXY, sigmaXY, sigmaTheta);
            ultimoTimestampAceito = inputs.timestampMegaTag1;
            Logger.recordOutput("Vision/Accepted", true);
            Logger.recordOutput("Vision/AcceptedSource", "MegaTag1");
        } else {
            Logger.recordOutput("Vision/Accepted", false);
            Logger.recordOutput("Vision/AcceptedSource", "None");
        }
    }

    private boolean medicaoValida(Pose2d pose, double timestamp, int tags, double distancia) {
        if (tags < Constants.Vision.TAGS_MINIMAS || !Double.isFinite(timestamp)
                || timestamp <= 0.0 || timestamp <= ultimoTimestampAceito
                || !Double.isFinite(pose.getX()) || !Double.isFinite(pose.getY())
                || !Double.isFinite(distancia) || distancia <= 0.0
                || distancia > Constants.Vision.DISTANCIA_MAXIMA_MT2_METROS) return false;
        if (Math.abs(drive.getYawVelocityRadPerSec())
                > Constants.Vision.VELOCIDADE_ANGULAR_MAXIMA_PARA_VISAO_RAD_S) return false;
        // Reject estimates far from the current odometry. The tolerance scales with range
        // and tag count; this guards against wrong field setup or transient bad solves.
        double erro = pose.getTranslation().getDistance(drive.getPose().getTranslation());
        double tolerancia = Math.max(1.0, distancia * (tags >= 2 ? 1.5 : 1.0));
        if (erro > tolerancia) return false;
        if (tags == 1 && inputs.ambiguidadeMinimaMegaTag2 > Constants.Vision.AMBIGUIDADE_MAXIMA_TAG_UNICA
                && inputs.ambiguidadeMinimaMegaTag2 > 0.0) return false;
        return !DriverStation.isEStopped();
    }
}
