package frc.robot.commands.features;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OffsetsConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

public class ResetOdometry extends Command {
    private static final double kP = 04.00;
    private static final double toleranciaGraus = 1.0;
    private static final double velocidadeAngularMaximaRadS = 1.5;
    private static final double sentidoRotacao = -1.0;

    private final Vision vision;
    private final Drive drive;

    public ResetOdometry(Vision vision, Drive drive) {
        this.vision = vision;
        this.drive = drive;
        addRequirements(vision, drive);
    }

    @Override
    public void execute() {
        if (!vision.temAprilTag()) {
            
            

        }


    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
