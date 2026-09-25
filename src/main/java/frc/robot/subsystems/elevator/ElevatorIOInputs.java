package frc.robot.subsystems.elevator;

public class ElevatorIOInputs {

    public double posicaoEncoderEsquerdo = 0.0;
    public double posicaoEncoderDireito = 0.0;
    public double velocidadeEncoderEsquerdo = 0.0;
    public double velocidadeEncoderDireito = 0.0;

    public double leituraBrutaEncoderAbsoluto = 0.0;
    public int frequenciaEncoderAbsoluto = 0;
    public boolean encoderAbsolutoConectado = false;

    public double tensaoMotorEsquerdo = 0.0;
    public double tensaoMotorDireito = 0.0;
    public double correnteMotorEsquerdo = 0.0;
    public double correnteMotorDireito = 0.0;
    public double temperaturaMotorEsquerdo = 0.0;
    public double temperaturaMotorDireito = 0.0;
}
