package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private final VisionIO io;
    private final VisionIO.VisionIOInputs entradas = new VisionIO.VisionIOInputs();

    public Vision(VisionIO io) {
        this.io = io;
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
        io.definirPipeline(pipeline); //não entendi
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
