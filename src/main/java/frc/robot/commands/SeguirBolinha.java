package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

public class SeguirBolinha extends Command {

    private final Vision vision;
    private final Drive drive;

    private static final double KP_ROTACAO = 0.04;
    private static final double ROTACAO_MAXIMA = 1.5;

    private static final double AREA_DESEJADA = 8.0;
    private static final double KP_FRENTE = 0.18;

    private static final double VELOCIDADE_MAXIMA = 1.5;
    private static final double VELOCIDADE_MINIMA = 0.20;

    public SeguirBolinha(Vision vision, Drive drive) {
        this.vision = vision;
        this.drive = drive;

        addRequirements(vision, drive);
    }

    @Override
    public void initialize() {
        vision.definirPipeline(0);
    }

    @Override
    public void execute() {

        if (!vision.temAlvo()) {
            drive.stop();
            return;
        }

        double tx = vision.pegarTx();
        double area = vision.pegarArea();
        double rotacao = MathUtil.clamp(-tx * KP_ROTACAO, -ROTACAO_MAXIMA, ROTACAO_MAXIMA);
        double erroArea = AREA_DESEJADA - area;
        if (erroArea <= 0.0) {
            drive.stop();
            return;
        }

        double frente = MathUtil.clamp(erroArea * KP_FRENTE, VELOCIDADE_MINIMA, VELOCIDADE_MAXIMA);

        drive.runVelocity(
            new ChassisSpeeds(frente, 0.0, rotacao)
        );
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
        vision.definirPipeline(1);
    }
}