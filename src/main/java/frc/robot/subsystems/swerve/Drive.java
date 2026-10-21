package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Degrees;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OffsetsConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroIOInputsAutoLogged;
import frc.robot.subsystems.swerve.gyro.GyroIOReal;
import frc.robot.subsystems.swerve.module.Module;
import frc.robot.subsystems.swerve.module.ModuleIO;
import frc.robot.subsystems.swerve.module.ModuleIOReal;
import frc.robot.subsystems.swerve.module.ModuleIOSim;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Swerve drivetrain subsystem.
 *
 * <p>Owns the four {@link Module}s, the gyro and the pose estimator. It consumes high-frequency
 * odometry samples from {@link SwerveOdometry}, fuses vision measurements, exposes chassis-level
 * control ({@link #runVelocity}) and configures PathPlanner for autonomous path following.
 */
public class Drive extends SubsystemBase {

  // ---------------------------------------------------------------------------
  // Constants
  // ---------------------------------------------------------------------------

  /** Odometry sampling frequency, in Hz (250 on CAN FD, 100 on classic CAN). */
  public static final double ODOMETRY_FREQUENCY =
      Constants.Swerve.CAN_BUS.isNetworkFD() ? 250.0 : 100.0;

  /** Guards the odometry queues shared between the main loop and the odometry thread. */
  static final Lock odometryLock = new ReentrantLock();

  // ---------------------------------------------------------------------------
  // State
  // ---------------------------------------------------------------------------

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Rotation2d hardwareGyroOffset =
      Rotation2d.fromDegrees(OffsetsConstants.Pigeon.YAW.in(Degrees));
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(getModuleTranslations());
  private final Alert gyroDisconnectedAlert =
      new Alert(
          "Gyro desconectado. O chassi está usando a cinemática como reserva.", AlertType.kError);

  private Rotation2d rawGyroRotation = new Rotation2d();
  private Rotation2d driverGyroOffset =
      Rotation2d.fromDegrees(OffsetsConstants.Driver.FRENTE_GRAUS);
  private SwerveModulePosition[] lastModulePositions = {
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition()
  };
  private final SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  // ---------------------------------------------------------------------------
  // Construction
  // ---------------------------------------------------------------------------

  /**
   * Creates the drive subsystem.
   *
   * @param gyroIO gyro hardware abstraction
   * @param frontLeft front-left module IO
   * @param frontRight front-right module IO
   * @param backLeft back-left module IO
   * @param backRight back-right module IO
   */
  public Drive(
      GyroIO gyroIO,
      ModuleIO frontLeft,
      ModuleIO frontRight,
      ModuleIO backLeft,
      ModuleIO backRight) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(frontLeft, 0, Constants.Swerve.FRONT_LEFT);
    modules[1] = new Module(frontRight, 1, Constants.Swerve.FRONT_RIGHT);
    modules[2] = new Module(backLeft, 2, Constants.Swerve.BACK_LEFT);
    modules[3] = new Module(backRight, 3, Constants.Swerve.BACK_RIGHT);

    // Usage reporting (AdvantageKit swerve template).
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start the odometry thread.
    SwerveOdometry.getInstance().start();

    try {
      configurarAutonomo();
    } catch (Exception e) {
      // A PathPlanner failure must never bring down teleop.
      DriverStation.reportError(
          "Falha ao configurar o autonomo: " + e.getMessage(), e.getStackTrace());
    }

    SmartDashboard.putNumber("Chassi/Massa kg", Constants.Swerve.MASSA_COM_BATERIA_E_BUMPERS_KG);
    SmartDashboard.putNumber(
        "Chassi/Velocidade máxima mps", Constants.Swerve.VELOCIDADE_MAXIMA_MPS);
    SmartDashboard.putNumber(
        "Chassi/Velocidade angular máxima radps",
        Constants.Swerve.VELOCIDADE_ANGULAR_MAXIMA_RAD_S);
  }

  /**
   * Creates a {@link Drive} with the IO implementations matching the current runtime mode.
   *
   * @return the drive subsystem for {@code REAL}, {@code SIM} or {@code REPLAY}
   */
  public static Drive criarSwerve() {
    return switch (Constants.MODO_ATUAL) {
      case REAL ->
          new Drive(
              new GyroIOReal(),
              new ModuleIOReal(TunerConstants.FrontLeft),
              new ModuleIOReal(TunerConstants.FrontRight),
              new ModuleIOReal(TunerConstants.BackLeft),
              new ModuleIOReal(TunerConstants.BackRight));
      case SIM ->
          new Drive(
              new GyroIO() {},
              new ModuleIOSim(TunerConstants.FrontLeft),
              new ModuleIOSim(TunerConstants.FrontRight),
              new ModuleIOSim(TunerConstants.BackLeft),
              new ModuleIOSim(TunerConstants.BackRight));
      case REPLAY ->
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
    };
  }

  /** Configures PathPlanner's {@link AutoBuilder} (holonomic path follower). */
  private void configurarAutonomo() {
    DCMotor motorTracao = DCMotor.getKrakenX60Foc(1);
    double raioRoda = Constants.Swerve.FRONT_LEFT.WheelRadius;
    double reducao = Constants.Swerve.FRONT_LEFT.DriveMotorGearRatio;

    RobotConfig config =
        new RobotConfig(
            Constants.Swerve.MASSA_COM_BATERIA_E_BUMPERS_KG,
            Constants.Swerve.MOMENTO_INERCIA_ESTIMADO_KG_M2,
            new ModuleConfig(
                raioRoda,
                motorTracao.freeSpeedRadPerSec / reducao * raioRoda,
                Constants.Autonomo.COEFICIENTE_ATRITO_RODA,
                motorTracao.withReduction(reducao),
                Constants.Swerve.LIMITE_CORRENTE_TRACAO_AMPERES,
                1),
            getModuleTranslations());

    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        (speeds, feedforwards) -> runVelocity(speeds),
        new PPHolonomicDriveController(
            new PIDConstants(Constants.Autonomo.KP_TRANSLACAO, 0.0, 0.0),
            new PIDConstants(Constants.Autonomo.KP_ROTACAO, 0.0, 0.0)),
        config,
        // Mirror paths for the red alliance.
        () -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
            == DriverStation.Alliance.Red,
        this);

    PathPlannerLogging.setLogActivePathCallback(
        path -> Logger.recordOutput("Autonomo/Trajetoria", path.toArray(new Pose2d[0])));
    PathPlannerLogging.setLogTargetPoseCallback(
        pose -> Logger.recordOutput("Autonomo/PoseAlvo", pose));
  }

  // ---------------------------------------------------------------------------
  // Periodic
  // ---------------------------------------------------------------------------

  @Override
  public void periodic() {
    // Update inputs while holding the lock so the odometry thread cannot modify the queues.
    odometryLock.lock();
    try {
      gyroIO.updateInputs(gyroInputs);
      Logger.processInputs("Drive/Gyro", gyroInputs);

      for (Module module : modules) {
        module.periodic();
      }
    } finally {
      odometryLock.unlock();
    }

    // Stop all modules while the robot is disabled.
    if (DriverStation.isDisabled()) {
      for (Module module : modules) {
        module.stop();
      }
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    atualizarOdometria();
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.MODO_ATUAL != Constants.Mode.SIM);

    Logger.recordOutput("Odometry/Robot", getPose());
    Logger.recordOutput("TESTE/PoseX", getPose().getX());
    Logger.recordOutput("TESTE/PoseY", getPose().getY());
    Logger.recordOutput("TESTE/PoseHeading", getPose().getRotation());
    Logger.recordOutput("TESTE/RawGyro", rawGyroRotation);
  }

  /**
   * Updates the pose estimator with every odometry sample collected since the last cycle.
   *
   * <p>Only sample indices available for every module are used. Queues can occasionally contain
   * different counts for one cycle; an early return here used to leave the odometry partially
   * updated and could corrupt the next delta calculation.
   */
  private void atualizarOdometria() {
    double[] timestamps = modules[0].getOdometryTimestamps();
    int sampleCount = timestamps.length;
    for (int modulo = 0; modulo < modules.length; modulo++) {
      sampleCount =
          Math.min(
              sampleCount,
              Math.min(
                  modules[modulo].getOdometryTimestamps().length,
                  modules[modulo].getOdometryPositions().length));
    }

    boolean gyroSamplesAvailable =
        gyroInputs.connected
            && gyroInputs.odometryYawPositions.length >= sampleCount
            && gyroInputs.odometryYawTimestamps.length >= sampleCount;

    for (int amostra = 0; amostra < sampleCount; amostra++) {
      SwerveModulePosition[] posicoes = new SwerveModulePosition[modules.length];
      SwerveModulePosition[] deltas = new SwerveModulePosition[modules.length];

      for (int modulo = 0; modulo < modules.length; modulo++) {
        posicoes[modulo] = modules[modulo].getOdometryPositions()[amostra];
        deltas[modulo] =
            new SwerveModulePosition(
                posicoes[modulo].distanceMeters - lastModulePositions[modulo].distanceMeters,
                posicoes[modulo].angle);
      }

      if (gyroSamplesAvailable) {
        rawGyroRotation = gyroInputs.odometryYawPositions[amostra].plus(hardwareGyroOffset);
      } else {
        // Fall back to wheel kinematics when the gyro is disconnected or its sample batch is
        // incomplete. This preserves odometry updates.
        Twist2d twist = kinematics.toTwist2d(deltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      poseEstimator.updateWithTime(timestamps[amostra], rawGyroRotation, posicoes);

      // Update the baseline only after all module deltas for this sample are computed.
      for (int modulo = 0; modulo < modules.length; modulo++) {
        lastModulePositions[modulo] = posicoes[modulo];
      }
    }

    Logger.recordOutput("Odometry/SampleCount", sampleCount);
    Logger.recordOutput("Odometry/GyroSamplesAvailable", gyroSamplesAvailable);
  }

  // ---------------------------------------------------------------------------
  // Control
  // ---------------------------------------------------------------------------

  /**
   * Drives the robot at the given robot-relative chassis speeds.
   *
   * <p>Speeds are clamped to the configured limits, discretized to compensate for the control loop
   * period, converted to module states and desaturated.
   *
   * @param speeds desired robot-relative chassis speeds
   */
  public void runVelocity(ChassisSpeeds speeds) {
    ChassisSpeeds velocidadesLimitadas =
        new ChassisSpeeds(
            MathUtil.clamp(
                speeds.vxMetersPerSecond,
                -Constants.Swerve.VELOCIDADE_MAXIMA_MPS,
                Constants.Swerve.VELOCIDADE_MAXIMA_MPS),
            MathUtil.clamp(
                speeds.vyMetersPerSecond,
                -Constants.Swerve.VELOCIDADE_MAXIMA_MPS,
                Constants.Swerve.VELOCIDADE_MAXIMA_MPS),
            MathUtil.clamp(
                speeds.omegaRadiansPerSecond,
                -Constants.Swerve.VELOCIDADE_ANGULAR_MAXIMA_RAD_S,
                Constants.Swerve.VELOCIDADE_ANGULAR_MAXIMA_RAD_S));

    ChassisSpeeds velocidadesDiscretas =
        ChassisSpeeds.discretize(velocidadesLimitadas, Constants.Swerve.PERIODO_CONTROLE_SEGUNDOS);

    SwerveModuleState[] estadosDesejados;
    if (Math.abs(velocidadesDiscretas.vxMetersPerSecond) < 1e-9
        && Math.abs(velocidadesDiscretas.vyMetersPerSecond) < 1e-9
        && Math.abs(velocidadesDiscretas.omegaRadiansPerSecond) < 1e-9) {
      // Zero command: keep the current wheel angles so the modules do not snap back to zero.
      SwerveModuleState[] estadosAtuais = getModuleStates();
      estadosDesejados = new SwerveModuleState[4];
      for (int i = 0; i < 4; i++) {
        estadosDesejados[i] = new SwerveModuleState(0.0, estadosAtuais[i].angle);
      }
    } else {
      estadosDesejados = kinematics.toSwerveModuleStates(velocidadesDiscretas);
    }

    SwerveDriveKinematics.desaturateWheelSpeeds(
        estadosDesejados, Constants.Swerve.VELOCIDADE_MAXIMA_MPS);

    Logger.recordOutput("SwerveStates/Setpoints", estadosDesejados);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", velocidadesDiscretas);

    // Each module optimizes its own setpoint in place.
    for (int i = 0; i < modules.length; i++) {
      modules[i].runSetpoint(estadosDesejados[i]);
    }

    Logger.recordOutput("SwerveStates/SetpointsOptimized", estadosDesejados);
  }

  /** Stops the drivetrain (zero chassis speeds). */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /** Placeholder for switching between normal and slow speed modes (not implemented yet). */
  public void mudarVelocidade() {}

  // ---------------------------------------------------------------------------
  // Pose and heading
  // ---------------------------------------------------------------------------

  /**
   * Resets the robot pose on the field (used by autonomous at the start of a path).
   *
   * @param pose the new pose
   */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, lastModulePositions, pose);
  }

  /** Resets the pose estimator's rotation to zero, keeping the current translation. */
  public void zerarOrientacaO() {
    poseEstimator.resetRotation(Rotation2d.kZero);
  }

  /** Marks the current heading as the driver's "forward" for field-relative control. */
  public void zeroHeading() {
    driverGyroOffset = getPose().getRotation();
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param poseVisao pose measured by vision
   * @param timestampSegundos capture timestamp, in seconds (FPGA time)
   * @param desvioPadraoXMetros X standard deviation, in meters
   * @param desvioPadraoYMetros Y standard deviation, in meters
   * @param desvioPadraoThetaRad heading standard deviation, in radians
   * @return the heading correction the measurement caused in the estimated pose
   */
  public Rotation2d adicionarMedicaoVisao(
      Pose2d poseVisao,
      double timestampSegundos,
      double desvioPadraoXMetros,
      double desvioPadraoYMetros,
      double desvioPadraoThetaRad) {
    Rotation2d headingAntes = getPose().getRotation();

    poseEstimator.addVisionMeasurement(
        poseVisao,
        timestampSegundos,
        VecBuilder.fill(desvioPadraoXMetros, desvioPadraoYMetros, desvioPadraoThetaRad));

    Rotation2d headingDepois = getPose().getRotation();
    Rotation2d correcaoHeading = headingDepois.minus(headingAntes);

    Logger.recordOutput("Odometry/VisionPose", poseVisao);
    Logger.recordOutput("Odometry/VisionHeadingCorrectionDeg", correcaoHeading.getDegrees());

    return correcaoHeading;
  }

  // ---------------------------------------------------------------------------
  // Accessors
  // ---------------------------------------------------------------------------

  /** Returns the estimated robot pose on the field. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the driver-relative robot rotation (estimated heading minus the driver offset). */
  public Rotation2d getRotation() {
    return getPose().getRotation().minus(driverGyroOffset);
  }

  /** Returns the estimated robot heading on the field. */
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  /** Returns the raw gyro rotation used by the pose estimator. */
  public Rotation2d getRawGyroRotation() {
    return rawGyroRotation;
  }

  /** Returns the yaw rate measured by the gyro, in rad/s. */
  public double getYawVelocityRadPerSec() {
    return gyroInputs.yawVelocityRadPerSec;
  }

  /** Returns the measured robot-relative chassis speeds. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the measured state (speed and angle) of each module. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] estados = new SwerveModuleState[4];
    for (int i = 0; i < modules.length; i++) {
      estados[i] = modules[i].getState();
    }
    return estados;
  }

  /** Returns the position of each module relative to the robot center (FL, FR, BL, BR). */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(
          Constants.Swerve.FRONT_LEFT.LocationX, Constants.Swerve.FRONT_LEFT.LocationY),
      new Translation2d(
          Constants.Swerve.FRONT_RIGHT.LocationX, Constants.Swerve.FRONT_RIGHT.LocationY),
      new Translation2d(
          Constants.Swerve.BACK_LEFT.LocationX, Constants.Swerve.BACK_LEFT.LocationY),
      new Translation2d(
          Constants.Swerve.BACK_RIGHT.LocationX, Constants.Swerve.BACK_RIGHT.LocationY)
    };
  }
}
