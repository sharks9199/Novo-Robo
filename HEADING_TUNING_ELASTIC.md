# Heading Tuning no Elastic

Os controles ficam em `/SmartDashboard/HeadingTuning/`.

Valores editaveis em tempo real:
- `kP`
- `kI`
- `kD`
- `MaxCorrectionDegPerSec`
- `PositionToleranceDeg`
- `VelocityToleranceDegPerSec`

Telemetria do controlador:
- `Active`
- `AtSetpoint`
- `CurrentHeadingDeg`
- `TargetHeadingDeg`
- `ErrorDeg`
- `PIDOutputDegPerSec`
- `AppliedOmegaDegPerSec`

Diagnostico do modo reto:
- `StraightDriveDetected`: true quando frente/tras esta usando Heading Hold.
- `LongitudinalSnapActive`: true quando pequeno ruido lateral esta sendo zerado.
- `LateralInput`: valor bruto do eixo lateral.
- `LateralApplied`: valor realmente usado pelo comando; deve ficar 0 no modo longitudinal travado.
- `YawVelocityDegPerSec`: velocidade angular medida pelo gyro.

O arquivo `assets/elasticOficial2026.json` possui a aba `Heading Tuning` com sliders de atualizacao continua.

Os valores em `Constants.Heading` sao os valores padrao usados ao iniciar. Depois do tuning no robo real, copie os valores finais do Elastic para `Constants.java`.

## Teste recomendado

Ao mandar somente frente/tras:
1. `LongitudinalSnapActive` deve ficar `true`.
2. `LateralApplied` deve ficar em `0` mesmo que `LateralInput` tenha pequeno ruido.
3. `StraightDriveDetected` e `Active` devem ficar `true` depois de qualquer giro manual terminar de estabilizar.
4. Se o gyro mostrar erro de heading acima da tolerancia, `PIDOutputDegPerSec` deve reagir e mudar ao alterar `kP`.

Se `LateralApplied = 0`, `ErrorDeg` permanece proximo de zero e mesmo assim a trajetoria fisica continua desviando, o problema nao e o HeadingController; verifique offsets/alinhamento dos modulos, velocidades reais e mecanica do drivetrain.
