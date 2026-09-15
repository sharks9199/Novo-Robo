package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

public class FollowFuel {
    
    private final Vision vision;
    private final Drive swerve;
    private final String nomeLimelight = "limelight-sharks";
    

    private final PIDController pid = new PIDController(0.001, 0, 0);

    public FollowFuel(Vision vision, Drive swerve){
        this.vision = vision;
        this.swerve = swerve;
        pid.setTolerance(0.5);
    }

    public void initialize(){
        pid.setSetpoint(0);
    }

    public void execute(){

        double tx_limelight = LimelightHelpers.getTX(nomeLimelight);
        
    }

}
