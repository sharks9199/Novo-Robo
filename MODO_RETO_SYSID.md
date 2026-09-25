# Modo RETO + SysId

## Controles

- **A (driver):** toggle do Modo RETO.
  - OFF -> ON: zera a orientacao exatamente pela mesma rotina do Start, desacelera, alinha os quatro modulos para a frente fisica definida por `Rotation2d.kZero` (zero calibrado dos módulos) e entra em RETO.
  - ON -> OFF: desliga HeadingController/Jerk pipeline e volta ao teleop normal.
- **Start (driver):** redefine o zero de orientacao. Se RETO estiver ativo, o alvo do heading volta a 0 graus.
- **X (driver):** mantém o alinhamento com Limelight que antes estava no A.
- **Right Stick X:** giro manual. No RETO o piloto sempre vence o HeadingController.

## Deadband direcional global

Funciona com o A ligado ou desligado.

- Deadband radial: 0.06.
- Se a intencao for frente/tras e o eixo lateral estiver ate 0.10, lateral vira exatamente 0.
- Se a intencao for esquerda/direita e o eixo longitudinal estiver ate 0.10, longitudinal vira exatamente 0.
- Histerese de saida em 0.14 evita liga/desliga perto do limite.

## Modo RETO

Pipeline:

1. JerkLimiter vetorial para translacao.
2. JerkLimiter angular.
3. HeadingController com `ProfiledPIDController`.
4. Snap automatico para 0 graus se o heading estiver dentro de +/-20 graus.
5. Giro manual desliga temporariamente o heading hold.
6. Ao soltar o giro, espera yaw estabilizar.
7. Se terminou dentro de +/-20 graus, volta para 0; fora disso segura o novo heading.
8. `ChassisSpeeds.discretize()`.
9. SwerveSetpointGenerator limita aceleracao de roda e taxa de steering.
10. Module optimization / cosine compensation no pipeline normal; no pipeline avancado a compensacao e feita no generator.

## Heading tuning no Elastic

Aba `RETO + SysId`:

- `kP`, `kI`, `kD`
- velocidade maxima do perfil
- aceleracao maxima do perfil
- tolerancia de posicao
- target / current / error
- setpoint e velocidade do perfil
- estado do RETO
- heading relativo
- janela +/-20 graus
- erro maximo de alinhamento dos modulos
- entrada bruta/processada do joystick

## SysId no Elastic

Na aba `RETO + SysId` existem quatro comandos:

- Quasistatic Forward
- Quasistatic Reverse
- Dynamic Forward
- Dynamic Reverse

Antes de cada teste o codigo desliga Modo RETO e HeadingController. Os modulos ficam alinhados e o drive recebe tensao aberta para caracterizacao. O log registra tensao, posicao, velocidade e corrente individual dos quatro modulos.

Valores iniciais:

- Ramp: 1.0 V/s
- Dynamic step: 4.0 V
- Timeout: 8 s

Use area livre e execute um teste de cada vez.

## Offsets aplicados

Valores informados pelo Phoenix Tuner em 25/09:

- FL = -0.13671875 rot
- FR = 0.044189453125 rot
- BL = -0.043212890625 rot
- BR = 0.380126953125 rot


> Nota de frames: `OffsetsConstants.Driver.FRENTE_GRAUS = -90°` continua sendo a referencia do driver. O alinhamento mecanico dos modulos usa 0° depois de aplicar os MagnetOffsets dos CANcoders; sao referencias diferentes.
