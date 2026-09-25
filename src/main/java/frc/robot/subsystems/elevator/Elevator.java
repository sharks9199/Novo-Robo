package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    public enum Estado {
        PARADO,
        MANUAL,
        PRESET,
        DESCIDA_INTERMEDIARIA,
        DESCIDA_FINAL,
        MANTENDO,
        CANCELADO
    }

    private enum PerfilAltura {
        NORMAL,
        INTERMEDIARIO,
        DESCIDA_FINAL
    }

    public enum Preset {
        H0(
            ElevatorConstants.Presets.H0_ALTURA_CM,
            ElevatorConstants.Presets.H0_ANGULO_GRAUS
        ),
        H1(
            ElevatorConstants.Presets.H1_ALTURA_CM,
            ElevatorConstants.Presets.H1_ANGULO_GRAUS
        ),
        H2(
            ElevatorConstants.Presets.H2_ALTURA_CM,
            ElevatorConstants.Presets.H2_ANGULO_GRAUS
        ),
        H3(
            ElevatorConstants.Presets.H3_ALTURA_CM,
            ElevatorConstants.Presets.H3_ANGULO_GRAUS
        );

        private final double alturaCentimetros;
        private final double anguloGraus;

        Preset(double alturaCentimetros, double anguloGraus) {
            this.alturaCentimetros = alturaCentimetros;
            this.anguloGraus = anguloGraus;
        }
    }

    private final ElevatorIO io;
    private final ElevatorIOInputs entradas = new ElevatorIOInputs();

    private final ProfiledPIDController controladorAltura = new ProfiledPIDController(
        ElevatorConstants.Altura.KP,
        ElevatorConstants.Altura.KI,
        ElevatorConstants.Altura.KD,
        restricoesNormais()
    );

    private final PIDController controladorGiro = new PIDController(
        ElevatorConstants.Giro.KP,
        ElevatorConstants.Giro.KI,
        ElevatorConstants.Giro.KD
    );

    private Estado estado = Estado.PARADO;
    private PerfilAltura perfilAltura = PerfilAltura.NORMAL;
    private String nomePreset = "Nenhum";

    private double posicaoEsquerdaCorrigida;
    private double posicaoDireitaCorrigida;
    private double alturaRotacoes;
    private double alturaRotacoesFiltrada;
    private boolean filtroInicializado;
    private boolean encodersAlturaValidos;

    private double alvoAlturaCentimetros;
    private double alvoAnguloGraus;
    private double alvoManutencaoAlturaCentimetros;
    private double alvoManutencaoAnguloGraus;

    private double direcaoManualAltura;
    private double direcaoManualGiro;
    private boolean gatilhoDireitoAnterior;

    private double tensaoAlturaAplicada;
    private double tensaoGiroAplicada;
    private double tensaoMotorEsquerdo;
    private double tensaoMotorDireito;

    public Elevator(ElevatorIO io) {
        this.io = io;

        controladorAltura.setTolerance(
            ElevatorConstants.Altura.TOLERANCIA_ROTACOES,
            ElevatorConstants.Altura.TOLERANCIA_VELOCIDADE
        );
        controladorGiro.setTolerance(ElevatorConstants.Giro.TOLERANCIA_GRAUS);

        if (ElevatorConstants.Giro.PERMITE_VOLTA_COMPLETA) {
            controladorGiro.enableContinuousInput(0.0, 360.0);
        }
    }

    public void atualizarEntradaOperador(int pov, double gatilhoDireito) {
        boolean gatilhoPressionado =
            gatilhoDireito > ElevatorConstants.Controle.LIMITE_GATILHO_DIREITO;

        if (gatilhoPressionado) {
            if (!gatilhoDireitoAnterior) {
                manterPosicaoAtual();
            }
            gatilhoDireitoAnterior = true;
            return;
        }

        if (gatilhoDireitoAnterior && estado == Estado.MANTENDO) {
            parar();
        }
        gatilhoDireitoAnterior = false;

        double novaDirecaoAltura = 0.0;
        double novaDirecaoGiro = 0.0;

        if (pov == 0) {
            novaDirecaoAltura = 1.0;
        } else if (pov == 180) {
            novaDirecaoAltura = -1.0;
        } else if (pov == 90) {
            novaDirecaoGiro = 1.0;
        } else if (pov == 270) {
            novaDirecaoGiro = -1.0;
        }

        boolean movimentoSolicitado =
            novaDirecaoAltura != 0.0 || novaDirecaoGiro != 0.0;

        if (movimentoSolicitado) {
            estado = Estado.MANUAL;
            nomePreset = "Nenhum";
            direcaoManualAltura = novaDirecaoAltura;
            direcaoManualGiro = novaDirecaoGiro;
            desativarControladores();
        } else if (estado == Estado.MANUAL) {
            direcaoManualAltura = 0.0;
            direcaoManualGiro = 0.0;
        }
    }

    public void irParaPreset(Preset preset) {
        alvoAlturaCentimetros = limitarAltura(preset.alturaCentimetros);
        alvoAnguloGraus = limitarAngulo(preset.anguloGraus);
        nomePreset = preset.name();
        direcaoManualAltura = 0.0;
        direcaoManualGiro = 0.0;

        if (
            preset == Preset.H0
            && obterAlturaCentimetros()
                > ElevatorConstants.Altura.ALTURA_INTERMEDIARIA_CM
                    + toleranciaAlturaCentimetros()
        ) {
            estado = Estado.DESCIDA_INTERMEDIARIA;
        } else if (preset == Preset.H0) {
            estado = Estado.DESCIDA_FINAL;
        } else {
            estado = Estado.PRESET;
        }

        prepararControleAutomatico();
    }

    public void manterPosicaoAtual() {
        alvoManutencaoAlturaCentimetros = obterAlturaCentimetros();
        alvoManutencaoAnguloGraus = obterAnguloGraus();
        direcaoManualAltura = 0.0;
        direcaoManualGiro = 0.0;
        nomePreset = "Manter posição";
        estado = Estado.MANTENDO;
        prepararControleAutomatico();
    }

    public void cancelarTudo() {
        estado = Estado.CANCELADO;
        nomePreset = "Nenhum";
        direcaoManualAltura = 0.0;
        direcaoManualGiro = 0.0;
        gatilhoDireitoAnterior = false;
        desativarControladores();
        aplicarSaidas(0.0, 0.0);
    }

    public void zerarEncodersAltura() {
        io.zerarEncodersAltura();
        estado = Estado.PARADO;
        nomePreset = "Nenhum";
        alturaRotacoes = 0.0;
        alturaRotacoesFiltrada = 0.0;
        filtroInicializado = true;
        direcaoManualAltura = 0.0;
        direcaoManualGiro = 0.0;
        desativarControladores();
        aplicarSaidas(0.0, 0.0);
    }

    public void parar() {
        estado = Estado.PARADO;
        nomePreset = "Nenhum";
        direcaoManualAltura = 0.0;
        direcaoManualGiro = 0.0;
        desativarControladores();
    }

    public Estado obterEstado() {
        return estado;
    }

    public double obterAlturaCentimetros() {
        double intervaloRotacoes =
            ElevatorConstants.Altura.ROTACOES_NA_ALTURA_MAXIMA
            - ElevatorConstants.Altura.ROTACOES_NA_ALTURA_MINIMA;

        if (Math.abs(intervaloRotacoes) < 1e-9) {
            return ElevatorConstants.Altura.MINIMA_CM;
        }

        double proporcao = (
            alturaRotacoesFiltrada
            - ElevatorConstants.Altura.ROTACOES_NA_ALTURA_MINIMA
        ) / intervaloRotacoes;

        return ElevatorConstants.Altura.MINIMA_CM
            + proporcao * (
                ElevatorConstants.Altura.MAXIMA_CM
                - ElevatorConstants.Altura.MINIMA_CM
            );
    }

    public double obterAnguloGraus() {
        double leitura = entradas.leituraBrutaEncoderAbsoluto;

        if (!Double.isFinite(leitura)) {
            return 0.0;
        }

        if (ElevatorConstants.EncoderAbsoluto.INVERTIDO) {
            leitura = 1.0 - leitura;
        }

        leitura -= ElevatorConstants.EncoderAbsoluto.OFFSET_ROTACOES;
        leitura -= Math.floor(leitura);
        return leitura * 360.0;
    }

    @Override
    public void periodic() {
        atualizarSensores();

        double tensaoAlturaDesejada = 0.0;
        double tensaoGiroDesejada = 0.0;

        switch (estado) {
            case MANUAL -> {
                tensaoAlturaDesejada = calcularAlturaManual();
                tensaoGiroDesejada = calcularGiroManual();

                if (
                    direcaoManualAltura == 0.0
                    && direcaoManualGiro == 0.0
                    && Math.abs(tensaoAlturaAplicada) < 0.02
                    && Math.abs(tensaoGiroAplicada) < 0.02
                ) {
                    estado = Estado.PARADO;
                }
            }
            case PRESET -> {
                tensaoAlturaDesejada = calcularAlturaAutomatica(
                    alvoAlturaCentimetros,
                    PerfilAltura.NORMAL
                );
                tensaoGiroDesejada = calcularGiroAutomatico(alvoAnguloGraus);
            }
            case DESCIDA_INTERMEDIARIA -> {
                tensaoAlturaDesejada = calcularAlturaAutomatica(
                    ElevatorConstants.Altura.ALTURA_INTERMEDIARIA_CM,
                    PerfilAltura.INTERMEDIARIO
                );
                tensaoGiroDesejada = calcularGiroAutomatico(alvoAnguloGraus);

                if (controladorAltura.atGoal()) {
                    estado = Estado.DESCIDA_FINAL;
                    prepararControleAutomatico();
                }
            }
            case DESCIDA_FINAL -> {
                tensaoAlturaDesejada = calcularAlturaAutomatica(
                    alvoAlturaCentimetros,
                    PerfilAltura.DESCIDA_FINAL
                );
                tensaoGiroDesejada = calcularGiroAutomatico(alvoAnguloGraus);
            }
            case MANTENDO -> {
                tensaoAlturaDesejada = calcularAlturaAutomatica(
                    alvoManutencaoAlturaCentimetros,
                    PerfilAltura.NORMAL
                );
                tensaoGiroDesejada = calcularGiroAutomatico(
                    alvoManutencaoAnguloGraus
                );
            }
            case PARADO, CANCELADO -> {
                tensaoAlturaAplicada = aplicarRampa(
                    tensaoAlturaAplicada,
                    0.0,
                    ElevatorConstants.Rampa.ALTURA_ACELERACAO_VOLTS_S,
                    ElevatorConstants.Rampa.ALTURA_DESACELERACAO_VOLTS_S
                );
                tensaoGiroAplicada = aplicarRampa(
                    tensaoGiroAplicada,
                    0.0,
                    ElevatorConstants.Rampa.GIRO_ACELERACAO_VOLTS_S,
                    ElevatorConstants.Rampa.GIRO_DESACELERACAO_VOLTS_S
                );
                aplicarSaidas(tensaoAlturaAplicada, tensaoGiroAplicada);
                publicarDados();
                return;
            }
        }

        aplicarSaidas(tensaoAlturaDesejada, tensaoGiroDesejada);
        publicarDados();
    }

    private void atualizarSensores() {
        io.atualizarEntradas(entradas);

        posicaoEsquerdaCorrigida = ElevatorConstants.Hardware.INVERTER_ENCODER_ESQUERDO
            ? -entradas.posicaoEncoderEsquerdo
            : entradas.posicaoEncoderEsquerdo;

        posicaoDireitaCorrigida = ElevatorConstants.Hardware.INVERTER_ENCODER_DIREITO
            ? -entradas.posicaoEncoderDireito
            : entradas.posicaoEncoderDireito;

        encodersAlturaValidos =
            Double.isFinite(posicaoEsquerdaCorrigida)
            && Double.isFinite(posicaoDireitaCorrigida);

        if (!encodersAlturaValidos) {
            return;
        }

        alturaRotacoes = (posicaoEsquerdaCorrigida + posicaoDireitaCorrigida) / 2.0;

        if (!filtroInicializado) {
            alturaRotacoesFiltrada = alturaRotacoes;
            filtroInicializado = true;
        } else {
            alturaRotacoesFiltrada += ElevatorConstants.Altura.INTENSIDADE_FILTRO
                * (alturaRotacoes - alturaRotacoesFiltrada);
        }
    }

    private double calcularAlturaManual() {
        desativarControladorAltura();

        double desejada = 0.0;
        if (direcaoManualAltura > 0.0) {
            desejada = ElevatorConstants.Altura.TENSAO_MANUAL_SUBIDA;
        } else if (direcaoManualAltura < 0.0) {
            desejada = -ElevatorConstants.Altura.TENSAO_MANUAL_DESCIDA;
        }

        tensaoAlturaAplicada = aplicarRampa(
            tensaoAlturaAplicada,
            desejada,
            ElevatorConstants.Rampa.ALTURA_ACELERACAO_VOLTS_S,
            ElevatorConstants.Rampa.ALTURA_DESACELERACAO_VOLTS_S
        );

        return aplicarLimitesAltura(tensaoAlturaAplicada);
    }

    private double calcularGiroManual() {
        desativarControladorGiro();

        double desejada = direcaoManualGiro
            * ElevatorConstants.Giro.TENSAO_MANUAL
            * ElevatorConstants.Giro.SENTIDO_TENSAO;

        if (!encoderAbsolutoValido()) {
            desejada = 0.0;
        }

        tensaoGiroAplicada = aplicarRampa(
            tensaoGiroAplicada,
            desejada,
            ElevatorConstants.Rampa.GIRO_ACELERACAO_VOLTS_S,
            ElevatorConstants.Rampa.GIRO_DESACELERACAO_VOLTS_S
        );

        return aplicarLimitesGiro(tensaoGiroAplicada);
    }

    private double calcularAlturaAutomatica(
        double alturaAlvoCentimetros,
        PerfilAltura novoPerfil
    ) {
        if (!encodersAlturaValidos) {
            return 0.0;
        }

        double alvoRotacoes = converterAlturaParaRotacoes(alturaAlvoCentimetros);

        if (
            novoPerfil != perfilAltura
            || Math.abs(controladorAltura.getGoal().position - alvoRotacoes) > 1e-9
        ) {
            perfilAltura = novoPerfil;
            controladorAltura.setConstraints(restricoesPara(novoPerfil));
            controladorAltura.reset(alturaRotacoesFiltrada);
            controladorAltura.setGoal(alvoRotacoes);
        }

        double saida = controladorAltura.calculate(alturaRotacoesFiltrada);

        if (
            alvoRotacoes > ElevatorConstants.Altura.ROTACOES_NA_ALTURA_MINIMA
                + ElevatorConstants.Altura.TOLERANCIA_ROTACOES
        ) {
            saida += ElevatorConstants.Altura.FF_VOLTS;
        }

        tensaoAlturaAplicada = MathUtil.clamp(
            saida,
            -ElevatorConstants.Altura.TENSAO_MAXIMA_AUTOMATICA,
            ElevatorConstants.Altura.TENSAO_MAXIMA_AUTOMATICA
        );

        return aplicarLimitesAltura(tensaoAlturaAplicada);
    }

    private double calcularGiroAutomatico(double anguloAlvoGraus) {
        if (!encoderAbsolutoValido()) {
            return 0.0;
        }

        double alvoLimitado = limitarAngulo(anguloAlvoGraus);
        controladorGiro.setSetpoint(alvoLimitado);

        double desejada = controladorGiro.calculate(obterAnguloGraus())
            * ElevatorConstants.Giro.SENTIDO_TENSAO;

        desejada = MathUtil.clamp(
            desejada,
            -ElevatorConstants.Giro.TENSAO_MAXIMA_AUTOMATICA,
            ElevatorConstants.Giro.TENSAO_MAXIMA_AUTOMATICA
        );

        tensaoGiroAplicada = aplicarRampa(
            tensaoGiroAplicada,
            desejada,
            ElevatorConstants.Rampa.GIRO_ACELERACAO_VOLTS_S,
            ElevatorConstants.Rampa.GIRO_DESACELERACAO_VOLTS_S
        );

        return aplicarLimitesGiro(tensaoGiroAplicada);
    }

    private void aplicarSaidas(double tensaoAltura, double tensaoGiro) {
        if (!Double.isFinite(tensaoAltura) || !Double.isFinite(tensaoGiro)) {
            tensaoAltura = 0.0;
            tensaoGiro = 0.0;
        }

        tensaoAlturaAplicada = MathUtil.clamp(
            tensaoAltura,
            -ElevatorConstants.Controle.TENSAO_MAXIMA_MOTORES,
            ElevatorConstants.Controle.TENSAO_MAXIMA_MOTORES
        );
        tensaoGiroAplicada = MathUtil.clamp(
            tensaoGiro,
            -ElevatorConstants.Controle.TENSAO_MAXIMA_MOTORES,
            ElevatorConstants.Controle.TENSAO_MAXIMA_MOTORES
        );

        double giroMotorEsquerdo = tensaoGiroAplicada
            * ElevatorConstants.Giro.FATOR_GIRO_MOTOR_ESQUERDO;
        double giroMotorDireito = tensaoGiroAplicada
            * ElevatorConstants.Giro.FATOR_GIRO_MOTOR_DIREITO;

        double esquerdaCalculada = giroMotorEsquerdo - tensaoAlturaAplicada;
        double direitaCalculada = giroMotorDireito + tensaoAlturaAplicada;
        double maiorMagnitude = Math.max(
            Math.abs(esquerdaCalculada),
            Math.abs(direitaCalculada)
        );

        double escala = maiorMagnitude > ElevatorConstants.Controle.TENSAO_MAXIMA_MOTORES
            ? ElevatorConstants.Controle.TENSAO_MAXIMA_MOTORES / maiorMagnitude
            : 1.0;

        tensaoMotorEsquerdo = esquerdaCalculada * escala;
        tensaoMotorDireito = direitaCalculada * escala;
        io.aplicarTensoes(tensaoMotorEsquerdo, tensaoMotorDireito);
    }

    private double aplicarLimitesAltura(double tensao) {
        if (!encodersAlturaValidos) {
            return 0.0;
        }

        if (
            alturaRotacoesFiltrada <= ElevatorConstants.Altura.ROTACOES_NA_ALTURA_MINIMA
            && tensao < 0.0
        ) {
            return 0.0;
        }

        if (
            alturaRotacoesFiltrada >= ElevatorConstants.Altura.ROTACOES_NA_ALTURA_MAXIMA
            && tensao > 0.0
        ) {
            return 0.0;
        }

        return tensao;
    }

    private double aplicarLimitesGiro(double tensao) {
        if (!encoderAbsolutoValido()) {
            return 0.0;
        }

        if (ElevatorConstants.Giro.PERMITE_VOLTA_COMPLETA) {
            return tensao;
        }

        double angulo = obterAnguloGraus();
        double sentidoFisico = tensao;

        if (
            angulo <= ElevatorConstants.Giro.ANGULO_MINIMO_GRAUS
            && sentidoFisico < 0.0
        ) {
            return 0.0;
        }

        if (
            angulo >= ElevatorConstants.Giro.ANGULO_MAXIMO_GRAUS
            && sentidoFisico > 0.0
        ) {
            return 0.0;
        }

        return tensao;
    }

    private boolean encoderAbsolutoValido() {
        double leitura = entradas.leituraBrutaEncoderAbsoluto;
        return entradas.encoderAbsolutoConectado
            && entradas.frequenciaEncoderAbsoluto
                >= ElevatorConstants.EncoderAbsoluto.FREQUENCIA_MINIMA_HZ
            && Double.isFinite(leitura)
            && leitura >= ElevatorConstants.EncoderAbsoluto.LEITURA_MINIMA_VALIDA
            && leitura <= ElevatorConstants.EncoderAbsoluto.LEITURA_MAXIMA_VALIDA;
    }

    private double converterAlturaParaRotacoes(double alturaCentimetros) {
        double alturaLimitada = limitarAltura(alturaCentimetros);
        double proporcao = (
            alturaLimitada - ElevatorConstants.Altura.MINIMA_CM
        ) / (
            ElevatorConstants.Altura.MAXIMA_CM
            - ElevatorConstants.Altura.MINIMA_CM
        );

        return ElevatorConstants.Altura.ROTACOES_NA_ALTURA_MINIMA
            + proporcao * (
                ElevatorConstants.Altura.ROTACOES_NA_ALTURA_MAXIMA
                - ElevatorConstants.Altura.ROTACOES_NA_ALTURA_MINIMA
            );
    }

    private double limitarAltura(double alturaCentimetros) {
        return MathUtil.clamp(
            alturaCentimetros,
            ElevatorConstants.Altura.MINIMA_CM,
            ElevatorConstants.Altura.MAXIMA_CM
        );
    }

    private double limitarAngulo(double anguloGraus) {
        if (ElevatorConstants.Giro.PERMITE_VOLTA_COMPLETA) {
            double normalizado = anguloGraus % 360.0;
            return normalizado < 0.0 ? normalizado + 360.0 : normalizado;
        }

        return MathUtil.clamp(
            anguloGraus,
            ElevatorConstants.Giro.ANGULO_MINIMO_GRAUS,
            ElevatorConstants.Giro.ANGULO_MAXIMO_GRAUS
        );
    }

    private double aplicarRampa(
        double valorAtual,
        double valorDesejado,
        double taxaAceleracao,
        double taxaDesaceleracao
    ) {
        if (valorAtual * valorDesejado < 0.0) {
            valorDesejado = 0.0;
        }

        boolean aumentandoMagnitude =
            Math.abs(valorDesejado) > Math.abs(valorAtual);
        double taxa = aumentandoMagnitude ? taxaAceleracao : taxaDesaceleracao;
        double variacaoMaxima = taxa * ElevatorConstants.Controle.PERIODO_SEGUNDOS;

        return valorAtual + MathUtil.clamp(
            valorDesejado - valorAtual,
            -variacaoMaxima,
            variacaoMaxima
        );
    }

    private void prepararControleAutomatico() {
        perfilAltura = PerfilAltura.NORMAL;
        controladorAltura.reset(alturaRotacoesFiltrada);
        controladorGiro.reset();
    }

    private void desativarControladores() {
        desativarControladorAltura();
        desativarControladorGiro();
    }

    private void desativarControladorAltura() {
        controladorAltura.reset(alturaRotacoesFiltrada);
    }

    private void desativarControladorGiro() {
        controladorGiro.reset();
    }

    private double toleranciaAlturaCentimetros() {
        return ElevatorConstants.Altura.TOLERANCIA_ROTACOES
            * (
                ElevatorConstants.Altura.MAXIMA_CM
                - ElevatorConstants.Altura.MINIMA_CM
            )
            / (
                ElevatorConstants.Altura.ROTACOES_NA_ALTURA_MAXIMA
                - ElevatorConstants.Altura.ROTACOES_NA_ALTURA_MINIMA
            );
    }

    private void publicarDados() {
        SmartDashboard.putString("Elevador/Estado", estado.name());
        SmartDashboard.putString("Elevador/Preset", nomePreset);
        SmartDashboard.putNumber("Elevador/Altura cm", obterAlturaCentimetros());
        SmartDashboard.putNumber("Elevador/Altura rotações", alturaRotacoesFiltrada);
        SmartDashboard.putNumber("Elevador/Ângulo graus", obterAnguloGraus());
        SmartDashboard.putBoolean("Elevador/Encoders altura válidos", encodersAlturaValidos);
        SmartDashboard.putBoolean("Elevador/Encoder absoluto válido", encoderAbsolutoValido());
        SmartDashboard.putNumber("Elevador/Tensão motor esquerdo", tensaoMotorEsquerdo);
        SmartDashboard.putNumber("Elevador/Tensão motor direito", tensaoMotorDireito);
        SmartDashboard.putNumber("Elevador/Corrente motor esquerdo", entradas.correnteMotorEsquerdo);
        SmartDashboard.putNumber("Elevador/Corrente motor direito", entradas.correnteMotorDireito);
    }

    private static TrapezoidProfile.Constraints restricoesNormais() {
        return new TrapezoidProfile.Constraints(
            ElevatorConstants.Altura.VELOCIDADE_MAXIMA_NORMAL,
            ElevatorConstants.Altura.ACELERACAO_MAXIMA_NORMAL
        );
    }

    private static TrapezoidProfile.Constraints restricoesPara(PerfilAltura perfil) {
        return switch (perfil) {
            case NORMAL -> restricoesNormais();
            case INTERMEDIARIO -> new TrapezoidProfile.Constraints(
                ElevatorConstants.Altura.VELOCIDADE_MAXIMA_INTERMEDIARIA,
                ElevatorConstants.Altura.ACELERACAO_MAXIMA_INTERMEDIARIA
            );
            case DESCIDA_FINAL -> new TrapezoidProfile.Constraints(
                ElevatorConstants.Altura.VELOCIDADE_MAXIMA_DESCIDA_FINAL,
                ElevatorConstants.Altura.ACELERACAO_MAXIMA_DESCIDA_FINAL
            );
        };
    }
}
