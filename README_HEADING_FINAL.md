# Heading Hold automatico - configuracao final

- Start redefine apenas o zero do driver/field-oriented.
- A pose global do `SwerveDrivePoseEstimator` nao e zerada pelo Start.
- Odometria continua sendo atualizada por gyro + modulos.
- Medicoes AprilTag/Limelight continuam entrando no PoseEstimator.
- O HeadingController atua somente quando a intencao do piloto e andar para frente ou para tras, sem strafe intencional e sem comando manual de giro.
- Pequenos desvios do eixo lateral do joystick sao tratados como ruido e o comando lateral aplicado vira zero.
- Existe histerese para impedir que o modo reto fique ligando/desligando perto do limite lateral.
- O motorista sempre tem prioridade ao usar o Right Stick: giro manual desativa imediatamente o HeadingController.
- Depois de um giro manual, o codigo espera a velocidade angular fisica e o SlewRateLimiter estabilizarem antes de capturar o novo heading.
- Parado, em strafe ou em diagonal intencional, o HeadingController fica desativado.
- Frente inicial do driver: `OffsetsConstants.FRENTE_GRAUS = -90.0`.

## Valores iniciais

- kP: 4.0
- kI: 0.0
- kD: 0.15
- correcao angular maxima: 120 deg/s
- tolerancia angular: 0.50 deg
- tolerancia de velocidade angular para considerar giro estabilizado: 4 deg/s
- entrada longitudinal minima para modo reto: 0.08
- lateral maxima para entrar no modo reto: 0.10
- lateral maxima para sair do modo reto: 0.14

Os ganhos do HeadingController continuam ajustaveis em tempo real pelo Elastic.
