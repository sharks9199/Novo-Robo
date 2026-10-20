package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;

public class VisionIOReal implements VisionIO{

  

    public VisionIOReal(){

    }

    public void updateInputs(VisionIOInputs inputs){ //Preencher para Visão.
        inputs.ambiguidadeMinimaMegaTag1 = 0;
        inputs.ambiguidadeMinimaMegaTag2 = 0;
        inputs.aprilTagID = -1;
        inputs.distanciaMediaTagsMegaTag1 = 0;
        inputs.distanciaMediaTagsMegaTag2 = 0;
        inputs.poseMegaTag1 = new Pose2d();
        inputs.poseMegaTag2 = new Pose2d();
        inputs.quantidadeTagsMegaTag1 = 0;
        inputs.quantidadeTagsMegaTag2 = 0;
        inputs.temAlvo = false;
        inputs.timestampMegaTag1 = 0;
        inputs.timestampMegaTag2 = 0;
    }



}
