package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OffsetsConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

public class AlinharComLimelight extends Command {
    private static final double kP = 0.04;
    private static final double toleranciaGraus = 1.0;
    private static final double velocidadeAngularMaximaRadS = 1.5;
    private static final double sentidoRotacao = -1.0;

    private final Vision vision;
    private final Drive drive;

    public AlinharComLimelight(Vision vision, Drive drive) {
        this.vision = vision;
        this.drive = drive;
        addRequirements(vision, drive);
    }

    @Override
    public void execute() {
        if (!vision.temAprilTag()) {
            drive.stop();
            return;
        }

        double erroHorizontalGraus = vision.pegarTx()
            - OffsetsConstants.Limelight.TX_ALVO_GRAUS;

        if (Math.abs(erroHorizontalGraus) <= toleranciaGraus) {
            drive.stop();
            return;
        }

        double velocidadeAngular = MathUtil.clamp(
            erroHorizontalGraus * kP * sentidoRotacao,
            -velocidadeAngularMaximaRadS,
            velocidadeAngularMaximaRadS
        );

        drive.runVelocity(new ChassisSpeeds(0.0, 0.0, velocidadeAngular));
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
