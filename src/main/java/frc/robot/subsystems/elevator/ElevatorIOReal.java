package frc.robot.subsystems.elevator;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ElevatorIOReal implements ElevatorIO {
    private final SparkMax motorEsquerdo = new SparkMax(
        ElevatorConstants.Hardware.ID_MOTOR_ESQUERDO,
        MotorType.kBrushless
    );

    private final SparkMax motorDireito = new SparkMax(
        ElevatorConstants.Hardware.ID_MOTOR_DIREITO,
        MotorType.kBrushless
    );

    private final RelativeEncoder encoderEsquerdo = motorEsquerdo.getEncoder();
    private final RelativeEncoder encoderDireito = motorDireito.getEncoder();

    private final DutyCycleEncoder encoderAbsoluto = new DutyCycleEncoder(
        ElevatorConstants.Hardware.PORTA_ENCODER_ABSOLUTO
    );

    private double tensaoEsquerdaAplicada;
    private double tensaoDireitaAplicada;

    public ElevatorIOReal() {
        motorEsquerdo.configure(
            criarConfiguracao(
                ElevatorConstants.Hardware.INVERTER_MOTOR_ESQUERDO
            ),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        motorDireito.configure(
            criarConfiguracao(
                ElevatorConstants.Hardware.INVERTER_MOTOR_DIREITO
            ),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        encoderAbsoluto.setConnectedFrequencyThreshold(
            ElevatorConstants.EncoderAbsoluto.FREQUENCIA_MINIMA_HZ
        );
    }

    @Override
    public void atualizarEntradas(ElevatorIOInputs entradas) {
        entradas.posicaoEncoderEsquerdo = encoderEsquerdo.getPosition();
        entradas.posicaoEncoderDireito = encoderDireito.getPosition();
        entradas.velocidadeEncoderEsquerdo = encoderEsquerdo.getVelocity();
        entradas.velocidadeEncoderDireito = encoderDireito.getVelocity();

        entradas.leituraBrutaEncoderAbsoluto = encoderAbsoluto.get();
        entradas.frequenciaEncoderAbsoluto = encoderAbsoluto.getFrequency();
        entradas.encoderAbsolutoConectado = encoderAbsoluto.isConnected();

        entradas.tensaoMotorEsquerdo = tensaoEsquerdaAplicada;
        entradas.tensaoMotorDireito = tensaoDireitaAplicada;
        entradas.correnteMotorEsquerdo = motorEsquerdo.getOutputCurrent();
        entradas.correnteMotorDireito = motorDireito.getOutputCurrent();
        entradas.temperaturaMotorEsquerdo = motorEsquerdo.getMotorTemperature();
        entradas.temperaturaMotorDireito = motorDireito.getMotorTemperature();
    }

    @Override
    public void aplicarTensoes(double tensaoMotorEsquerdo, double tensaoMotorDireito) {
        tensaoEsquerdaAplicada = MathUtil.clamp(
            tensaoMotorEsquerdo,
            -ElevatorConstants.Controle.TENSAO_MAXIMA_MOTORES,
            ElevatorConstants.Controle.TENSAO_MAXIMA_MOTORES
        );
        tensaoDireitaAplicada = MathUtil.clamp(
            tensaoMotorDireito,
            -ElevatorConstants.Controle.TENSAO_MAXIMA_MOTORES,
            ElevatorConstants.Controle.TENSAO_MAXIMA_MOTORES
        );

        motorEsquerdo.setVoltage(tensaoEsquerdaAplicada);
        motorDireito.setVoltage(tensaoDireitaAplicada);
    }

    @Override
    public void zerarEncodersAltura() {
        encoderEsquerdo.setPosition(0.0);
        encoderDireito.setPosition(0.0);
    }

    private SparkMaxConfig criarConfiguracao(boolean invertido) {
        SparkMaxConfig configuracao = new SparkMaxConfig();
        configuracao
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ElevatorConstants.Hardware.LIMITE_CORRENTE_AMPERES)
            .inverted(invertido);
        return configuracao;
    }
}
