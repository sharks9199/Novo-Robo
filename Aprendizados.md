# Aprendizados

## Swerve e Odometria

### getPose2d
Serve para pegar a posição estimada atual do robô no campo.

A pose possui:
- X: posição horizontal na arena.
- Y: posição vertical na arena.
- Rotação: direção que o robô está olhando.

Ela vem do `SwerveDrivePoseEstimator`.

Fluxo:

Módulos Swerve + Gyro
→ Odometria
→ Pose2d
→ AdvantageScope

### SwerveDrivePoseEstimator
É o objeto responsável por juntar:
- movimento dos módulos;
- rotação do gyro;
- correções da visão.

Ele calcula onde o robô acredita estar.

### addVisionMeasurement
Adiciona uma correção externa na odometria.

Exemplo:
- O swerve estima que está em X=3.
- A Limelight vê AprilTags e calcula X=3.2.
- A estimativa é corrigida.

### Vision
Responsável por receber dados da câmera.

Ela não movimenta o robô. Ela apenas entrega informações.

### Limelight MegaTag2
Usa AprilTags para calcular a posição do robô na arena.

Fluxo:

AprilTags
→ Limelight
→ PoseEstimate
→ addVisionMeasurement
→ Odometria

### Pipeline 1
Usada para AprilTags.

Objetivo:
- localizar o robô;
- corrigir a pose.

### Pipeline 0
Usada para detectar objetos/gamepieces.

Objetivo:
- pegar TX/TY;
- controlar o movimento até a peça.

### AdvantageScope
Recebe a Pose2d publicada pelo robô.

Mostra:
- posição;
- rotação;
- movimento no campo.

## Fluxo geral do robô

Controle:

Joystick
→ Command
→ Drive
→ Swerve Modules
→ Motores


Localização:

Encoders dos módulos
+ Gyro
→ Odometry
+ Limelight AprilTags
→ Pose2d
→ AdvantageScope

Detecção:

Limelight Pipeline 0
→ TX/TY da peça
→ Command SeguirBolinha
→ ChassisSpeeds
→ Drive
