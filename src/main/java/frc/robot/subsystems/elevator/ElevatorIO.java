package frc.robot.subsystems.elevator;

public interface ElevatorIO {

    void atualizarEntradas(ElevatorIOInputs entradas);

    void aplicarTensoes(double tensaoMotorEsquerdo, double tensaoMotorDireito);

    void zerarEncodersAltura();
}
