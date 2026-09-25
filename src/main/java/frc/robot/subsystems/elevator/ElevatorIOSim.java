package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;

public class ElevatorIOSim implements ElevatorIO {
    private double tensaoMotorEsquerdo;
    private double tensaoMotorDireito;
    private double posicaoEncoderEsquerdo;
    private double posicaoEncoderDireito;
    private double anguloGiroGraus = 90.0;

    @Override
    public void atualizarEntradas(ElevatorIOInputs entradas) {
        double tensaoAltura = (tensaoMotorDireito - tensaoMotorEsquerdo) / 2.0;
        double tensaoGiro = (tensaoMotorDireito + tensaoMotorEsquerdo) / 2.0;

        double velocidadeAlturaRotacoesPorSegundo = tensaoAltura * 2.0;
        double velocidadeGiroGrausPorSegundo = tensaoGiro * 18.0;

        posicaoEncoderEsquerdo -= velocidadeAlturaRotacoesPorSegundo
            * ElevatorConstants.Controle.PERIODO_SEGUNDOS;
        posicaoEncoderDireito += velocidadeAlturaRotacoesPorSegundo
            * ElevatorConstants.Controle.PERIODO_SEGUNDOS;
        anguloGiroGraus += velocidadeGiroGrausPorSegundo
            * ElevatorConstants.Controle.PERIODO_SEGUNDOS;

        anguloGiroGraus %= 360.0;
        if (anguloGiroGraus < 0.0) {
            anguloGiroGraus += 360.0;
        }

        entradas.posicaoEncoderEsquerdo = posicaoEncoderEsquerdo;
        entradas.posicaoEncoderDireito = posicaoEncoderDireito;
        entradas.velocidadeEncoderEsquerdo = -velocidadeAlturaRotacoesPorSegundo * 60.0;
        entradas.velocidadeEncoderDireito = velocidadeAlturaRotacoesPorSegundo * 60.0;
        entradas.leituraBrutaEncoderAbsoluto = anguloGiroGraus / 360.0;
        entradas.frequenciaEncoderAbsoluto = 244;
        entradas.encoderAbsolutoConectado = true;
        entradas.tensaoMotorEsquerdo = tensaoMotorEsquerdo;
        entradas.tensaoMotorDireito = tensaoMotorDireito;
        entradas.correnteMotorEsquerdo = Math.abs(tensaoMotorEsquerdo) * 2.0;
        entradas.correnteMotorDireito = Math.abs(tensaoMotorDireito) * 2.0;
        entradas.temperaturaMotorEsquerdo = 25.0;
        entradas.temperaturaMotorDireito = 25.0;
    }

    @Override
    public void aplicarTensoes(double tensaoMotorEsquerdo, double tensaoMotorDireito) {
        this.tensaoMotorEsquerdo = MathUtil.clamp(
            tensaoMotorEsquerdo,
            -ElevatorConstants.Controle.TENSAO_MAXIMA_MOTORES,
            ElevatorConstants.Controle.TENSAO_MAXIMA_MOTORES
        );
        this.tensaoMotorDireito = MathUtil.clamp(
            tensaoMotorDireito,
            -ElevatorConstants.Controle.TENSAO_MAXIMA_MOTORES,
            ElevatorConstants.Controle.TENSAO_MAXIMA_MOTORES
        );
    }

    @Override
    public void zerarEncodersAltura() {
        posicaoEncoderEsquerdo = 0.0;
        posicaoEncoderDireito = 0.0;
    }
}
