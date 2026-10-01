# Correções de odometria e HeadingController

## Ajuste da frente do controle
Em `src/main/java/frc/robot/OffsetsConstants.java`, altere `OffsetsConstants.Driver.FRENTE_GRAUS`
para `0.0`, `90.0`, `-90.0`, `180.0` ou `-180.0`, conforme o alinhamento do joystick com o campo.
Esse offset é aplicado apenas por `Drive.getRotation()` na condução field-relative; não é aplicado
às leituras do Pigeon, aos encoders, nem à pose estimada pela odometria.

## Atualização da odometria
- `Module.periodic()` só monta amostras completas (timestamp, posição da roda e ângulo do módulo).
- `Drive.atualizarOdometria()` usa a quantidade de amostras comum a todos os módulos e não abandona
  uma atualização no meio depois de já ter alterado os deltas.
- Se o lote de amostras do giroscópio estiver incompleto ou o gyro estiver desconectado, a rotação
  é estimada pela cinemática das rodas nesse lote.
- São registrados `Odometry/SampleCount` e `Odometry/GyroSamplesAvailable` para diagnóstico.

## Ghost do HeadingController
`DriveCommands` publica `HeadingController/Ghost` como uma `Pose2d`: usa a posição estimada atual
e a orientação-alvo do controlador enquanto ativo; quando inativo, usa a orientação medida atual.
O layout `assets/advantageScopeLayout2026.json` inclui essa fonte como Ghost azul no painel 2D Field.

## Validação
Compile com o Gradle/WPILib da versão instalada e valide primeiro em simulação. Antes de testar no
robô real, suspenda as rodas e confirme individualmente frente/trás, lateral, giro e sentido dos
encoders. A calibração física dos offsets dos módulos e do Pigeon continua sendo específica do robô.
