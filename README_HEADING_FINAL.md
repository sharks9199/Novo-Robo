# Heading Hold automatico - configuracao final

- Start redefine apenas o zero do driver/field-oriented.
- A pose global do SwerveDrivePoseEstimator nao e zerada pelo Start.
- Odometria continua sendo atualizada por gyro + modulos.
- Medicoes AprilTag/Limelight continuam entrando no PoseEstimator.
- HeadingController e automatico durante translacao sem comando manual de giro.
- O motorista sempre tem prioridade ao usar o Right Stick.
- Parado, o HeadingController fica desativado para nao girar sozinho.
- Vale para qualquer vetor de translacao: frente, tras, esquerda, direita e diagonais.
- Frente inicial do driver: OffsetsConstants.FRENTE_GRAUS = -90.0.
