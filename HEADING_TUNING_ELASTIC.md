# Heading Tuning no Elastic

Os controles ficam em `/SmartDashboard/HeadingTuning/`.

Valores editáveis em tempo real:
- `kP`
- `kI`
- `kD`
- `MaxCorrectionDegPerSec`
- `PositionToleranceDeg`
- `VelocityToleranceDegPerSec`

Telemetria:
- `Active`
- `AtSetpoint`
- `CurrentHeadingDeg`
- `TargetHeadingDeg`
- `ErrorDeg`
- `PIDOutputDegPerSec`
- `AppliedOmegaDegPerSec`

O arquivo `assets/elasticOficial2026.json` já possui a aba `Heading Tuning` com sliders de atualização contínua.

Os valores em `Constants.Heading` continuam sendo os valores padrão usados ao iniciar. Depois de terminar o tuning, copie os valores finais do Elastic para `Constants.java`.
