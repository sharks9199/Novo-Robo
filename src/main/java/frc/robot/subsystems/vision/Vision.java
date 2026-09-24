package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase {
    private final VisionIO io;
    private final VisionIO.VisionIOInputs entradas = new VisionIO.VisionIOInputs();
    private final VisionIOLimelight limelight;

    public Vision(VisionIO io) {
        this.io = io;
        this.limelight = io instanceof VisionIOLimelight ? (VisionIOLimelight) io : null;
    }

    public boolean temAlvo() {
        return entradas.temAlvo;
    }

    public double pegarArea(){
        return entradas.area;
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

    public LimelightHelpers.PoseEstimate pegarPoseMegaTag2() {
        return limelight != null ? limelight.pegarPoseMegaTag2() : new LimelightHelpers.PoseEstimate();
    }

    public void definirPipeline(int pipeline) {
        io.definirPipeline(pipeline);
    }

    @Override
    public void periodic() {
        io.updateInputs(entradas);
        SmartDashboard.putBoolean("Limelight/Tem alvo", temAlvo());
        SmartDashboard.putBoolean("Limelight/Tem AprilTag", temAprilTag());
        SmartDashboard.putNumber("Limelight/AprilTag ID", pegarAprilTagId());
        SmartDashboard.putNumber("Limelight/TX", pegarTx());
        SmartDashboard.putNumber("Limelight/TY", pegarTy());
    }
}
