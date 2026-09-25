package frc.robot.subsystems.vision;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
    public record MedicaoVisao(
        Pose2d pose,
        double timestampSegundos,
        double desvioPadraoXMetros,
        double desvioPadraoYMetros,
        double desvioPadraoThetaRad
    ) {}

    private final VisionIO io;
    private final VisionIO.VisionIOInputs entradas = new VisionIO.VisionIOInputs();
    private final Supplier<Rotation2d> headingSupplier;
    private final DoubleSupplier yawRateRadSSupplier;
    private final Consumer<MedicaoVisao> consumidorMedicao;

    public Vision(
        VisionIO io,
        Supplier<Rotation2d> headingSupplier,
        DoubleSupplier yawRateRadSSupplier,
        Consumer<MedicaoVisao> consumidorMedicao
    ) {
        this.io = io;
        this.headingSupplier = headingSupplier;
        this.yawRateRadSSupplier = yawRateRadSSupplier;
        this.consumidorMedicao = consumidorMedicao;
    }

    public boolean temAlvo() {
        return entradas.temAlvo;
    }

    public boolean temAprilTag() {
        return entradas.temAlvo && entradas.aprilTagId > 0;
    }

    public double pegarTx() {
        return entradas.tx;
    }

    public double pegarTy() {
        return entradas.ty;
    }

    public int pegarAprilTagId() {
        return entradas.aprilTagId;
    }

    public void definirPipeline(int pipeline) {
        io.definirPipeline(pipeline);
    }

    @Override
    public void periodic() {
        Rotation2d headingAtual = headingSupplier.get();
        double yawRateRadS = yawRateRadSSupplier.getAsDouble();

        io.definirOrientacaoRobo(
            headingAtual.getDegrees(),
            Units.radiansToDegrees(yawRateRadS)
        );
        io.updateInputs(entradas);

        boolean mt1Aceito = processarMegaTag1(yawRateRadS);
        if (!mt1Aceito) {
            processarMegaTag2(yawRateRadS);
        }

        SmartDashboard.putBoolean("Limelight/Tem alvo", temAlvo());
        SmartDashboard.putBoolean("Limelight/Tem AprilTag", temAprilTag());
        SmartDashboard.putNumber("Limelight/AprilTag ID", pegarAprilTagId());
        SmartDashboard.putNumber("Limelight/TX", pegarTx());
        SmartDashboard.putNumber("Limelight/TY", pegarTy());
        SmartDashboard.putNumber(
            "Limelight/MT1 Tags",
            entradas.quantidadeTagsMegaTag1
        );
        SmartDashboard.putNumber(
            "Limelight/MT2 Tags",
            entradas.quantidadeTagsMegaTag2
        );
    }

    private boolean processarMegaTag1(double yawRateRadS) {
        if (!medicaoBasicaValida(
            entradas.poseMegaTag1,
            entradas.timestampMegaTag1,
            entradas.quantidadeTagsMegaTag1,
            entradas.distanciaMediaTagsMegaTag1,
            yawRateRadS
        )) {
            return false;
        }

        boolean multiplasTags = entradas.quantidadeTagsMegaTag1 >= 2;
        boolean tagUnicaConfiavel =
            entradas.quantidadeTagsMegaTag1 == 1
            && entradas.distanciaMediaTagsMegaTag1
                <= Constants.Vision.DISTANCIA_MAXIMA_TAG_UNICA_HEADING_METROS
            && entradas.ambiguidadeMinimaMegaTag1
                <= Constants.Vision.AMBIGUIDADE_MAXIMA_TAG_UNICA_HEADING;

        if (!multiplasTags && !tagUnicaConfiavel) {
            return false;
        }

        double desvioXY = multiplasTags
            ? Constants.Vision.MT1_DESVIO_XY_MULTIPLAS_TAGS_METROS
            : Constants.Vision.MT1_DESVIO_XY_TAG_UNICA_METROS;

        double desvioTheta = multiplasTags
            ? Constants.Vision.MT1_DESVIO_THETA_MULTIPLAS_TAGS_RAD
            : Constants.Vision.MT1_DESVIO_THETA_TAG_UNICA_RAD;

        consumidorMedicao.accept(
            new MedicaoVisao(
                entradas.poseMegaTag1,
                entradas.timestampMegaTag1,
                desvioXY,
                desvioXY,
                desvioTheta
            )
        );

        Logger.recordOutput("Vision/MT1/Aceito", true);
        Logger.recordOutput("Vision/MT1/Pose", entradas.poseMegaTag1);
        Logger.recordOutput("Vision/MT1/TagCount", entradas.quantidadeTagsMegaTag1);
        Logger.recordOutput("Vision/MT1/Ambiguidade", entradas.ambiguidadeMinimaMegaTag1);
        return true;
    }

    private void processarMegaTag2(double yawRateRadS) {
        if (!medicaoBasicaValida(
            entradas.poseMegaTag2,
            entradas.timestampMegaTag2,
            entradas.quantidadeTagsMegaTag2,
            entradas.distanciaMediaTagsMegaTag2,
            yawRateRadS
        )) {
            Logger.recordOutput("Vision/MT2/Aceito", false);
            return;
        }

        double desvioXY = entradas.quantidadeTagsMegaTag2 >= 2
            ? Constants.Vision.MT2_DESVIO_XY_MULTIPLAS_TAGS_METROS
            : Constants.Vision.MT2_DESVIO_XY_TAG_UNICA_METROS;

        consumidorMedicao.accept(
            new MedicaoVisao(
                entradas.poseMegaTag2,
                entradas.timestampMegaTag2,
                desvioXY,
                desvioXY,
                Constants.Vision.MT2_DESVIO_THETA_RAD
            )
        );

        Logger.recordOutput("Vision/MT2/Aceito", true);
        Logger.recordOutput("Vision/MT2/Pose", entradas.poseMegaTag2);
        Logger.recordOutput("Vision/MT2/TagCount", entradas.quantidadeTagsMegaTag2);
    }

    private boolean medicaoBasicaValida(
        Pose2d pose,
        double timestampSegundos,
        int quantidadeTags,
        double distanciaMediaTags,
        double yawRateRadS
    ) {
        if (quantidadeTags <= 0 || timestampSegundos <= 0.0) {
            return false;
        }

        if (!Double.isFinite(pose.getX()) || !Double.isFinite(pose.getY())) {
            return false;
        }

        if (
            distanciaMediaTags <= 0.0
            || distanciaMediaTags > Constants.Vision.DISTANCIA_MAXIMA_VISAO_METROS
        ) {
            return false;
        }

        return Math.abs(yawRateRadS)
            <= Constants.Vision.VELOCIDADE_ANGULAR_MAXIMA_PARA_VISAO_RAD_S;
    }
}
