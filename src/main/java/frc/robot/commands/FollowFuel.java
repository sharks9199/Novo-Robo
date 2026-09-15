package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

public class FollowFuel extends Command{
    
    private final Vision vision;
    private final Drive swerve;
    private final String nomeLimelight = "limelight-sharks";
    

    private final PIDController pid = new PIDController(1.0, 0, 0);

    public FollowFuel(Vision vision, Drive swerve){
        this.vision = vision;
        this.swerve = swerve;
        pid.setTolerance(0.5);
    }

    public void initialize(){
        pid.setSetpoint(0);
    }

    public void girarAteFuel(){
        double tx_limelight = LimelightHelpers.getTX(nomeLimelight);
        double output = pid.calculate(tx_limelight);
        swerve.runVelocity(new ChassisSpeeds(0, 0, output));
        
    }
    public void andarAteFuel(){
        pid.setSetpoint(0);
        double ty_limelight = LimelightHelpers.getTY(nomeLimelight);
        double output = pid.calculate(ty_limelight);
        swerve.runVelocity(new ChassisSpeeds(0, -output, 0));
    }

//teste
    public void execute(){
        girarAteFuel();
        andarAteFuel();
    }
    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }

}
