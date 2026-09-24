package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOReal;

public class SetStraightMovement extends Command{

    private final Drive swerve;
    private final PIDController pid;

    private double setpointDeAngulo;

    private static final double velocidadeAngularMaximaRadS = 1.5;

    public SetStraightMovement(Drive swerve){
        this.swerve = swerve;
        this.pid = new PIDController(Constants.kPTurn, Constants.kITurn, Constants.kDTurn);
        this.pid.setTolerance(1.5);
        this.pid.enableContinuousInput(-180.0, 180.0);

        addRequirements(swerve);
    }

    //verificar ângulo atual


    @Override
    public void initialize() {
        setpointDeAngulo = decidir_angulo();
        pid.reset();
    }

    public double decidir_angulo(){

        List<Integer> posicoes = new ArrayList<>();
///talvez seja melhor usar a lista...
        posicoes.add(0);
        posicoes.add(90);
        posicoes.add(180);
        posicoes.add(-180);
        posicoes.add(-90);

        double guinadaAtual = retornarAngulo();

        int maisProximo = posicoes.get(0);

        for (int posicao : posicoes) {

            if (Math.abs(guinadaAtual - posicao)
                    < Math.abs(guinadaAtual - maisProximo)) {

                maisProximo = posicao;
            }
        }
        return maisProximo;
    }

    public double retornarAngulo(){
        Rotation2d angulo = GyroIOReal.pigeon.getRotation2d();
        double anguloContinuo = angulo.getDegrees();
        double anguloNormalizado = MathUtil.inputModulus(anguloContinuo, -180.0, 180.0);

        return anguloNormalizado;

    }

    @Override
    public void execute(){

        double output = MathUtil.clamp(pid.calculate(retornarAngulo(), setpointDeAngulo), -velocidadeAngularMaximaRadS, velocidadeAngularMaximaRadS);
        swerve.runVelocity(new ChassisSpeeds(0.0, 0.0, output));
    }

    @Override
    public boolean isFinished() {
        return pid.atSetpoint();
    
    }

    @Override
    public void end(boolean interrupted) {
        swerve.runVelocity(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    


}