package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

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
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.OffsetsConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.control.SwerveSetpointGenerator;

public class Drive extends SubsystemBase {
    static final double ODOMETRY_FREQUENCY =
        TunerConstants.CAN_BUS.isNetworkFD() ? 250.0 : 100.0;

    static final Lock odometryLock = new ReentrantLock();

    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4];
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        getModuleTranslations()
    );
    private final Alert gyroDisconnectedAlert = new Alert(
        "Gyro desconectado. O chassi está usando a cinemática como reserva.",
        AlertType.kError
    );

    private Rotation2d rawGyroRotation = new Rotation2d();
    private Rotation2d driverGyroOffset = Rotation2d.fromDegrees(OffsetsConstants.Driver.FRENTE_GRAUS);
    private SwerveModulePosition[] lastModulePositions = {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
    };
    private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
        kinematics,
        rawGyroRotation,
        lastModulePositions,
        new Pose2d()
    );
    private final SwerveSetpointGenerator advancedSetpointGenerator =
        new SwerveSetpointGenerator();
    private final SysIdRoutine sysIdRoutine;

    public Drive(
        GyroIO gyroIO,
        ModuleIO frontLeft,
        ModuleIO frontRight,
        ModuleIO backLeft,
        ModuleIO backRight
    ) {
        this.gyroIO = gyroIO;
        modules[0] = new Module(frontLeft, 0, TunerConstants.FrontLeft);
        modules[1] = new Module(frontRight, 1, TunerConstants.FrontRight);
        modules[2] = new Module(backLeft, 2, TunerConstants.BackLeft);
        modules[3] = new Module(backRight, 3, TunerConstants.BackRight);

        HAL.report(
            tResourceType.kResourceType_RobotDrive,
            tInstances.kRobotDriveSwerve_AdvantageKit
        );
        PhoenixOdometryThread.getInstance().start();

        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(Constants.SysId.RAMP_RATE_VOLTS_POR_SEGUNDO).per(Second),
                Volts.of(Constants.SysId.STEP_VOLTS),
                Seconds.of(Constants.SysId.TIMEOUT_SEGUNDOS),
                state -> {
                    Logger.recordOutput("Drive/SysId/State", state.toString());
                    SmartDashboard.putString("SysId/State", state.toString());
                }
            ),
            new SysIdRoutine.Mechanism(
                voltage -> runDriveCharacterization(voltage.in(Volts)),
                this::logSysId,
                this
            )
        );

        SmartDashboard.putString("SysId/State", "none");
        SmartDashboard.putNumber(
            "Chassi/Massa kg",
            Constants.Chassi.MASSA_COM_BATERIA_E_BUMPERS_KG
        );
        SmartDashboard.putNumber(
            "Chassi/Velocidade máxima mps",
            Constants.Chassi.VELOCIDADE_MAXIMA_MPS
        );
        SmartDashboard.putNumber(
            "Chassi/Velocidade angular máxima radps",
            Constants.Chassi.VELOCIDADE_ANGULAR_MAXIMA_RAD_S
        );
    }
    
    public void zerarOrientacaO(){

        poseEstimator.resetRotation(Rotation2d.kZero);
    }

    @Override
    public void periodic() {
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

        if (DriverStation.isDisabled()) {
            for (Module module : modules) {
                module.stop();
            }
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }

        atualizarOdometria();
        gyroDisconnectedAlert.set(
            !gyroInputs.connected && Constants.MODO_ATUAL != Constants.Mode.SIM
        );
    }

    public void runVelocity(ChassisSpeeds speeds) {
        ChassisSpeeds velocidadesLimitadas = new ChassisSpeeds(
            MathUtil.clamp(
                speeds.vxMetersPerSecond,
                -Constants.Chassi.VELOCIDADE_MAXIMA_MPS,
                Constants.Chassi.VELOCIDADE_MAXIMA_MPS
            ),
            MathUtil.clamp(
                speeds.vyMetersPerSecond,
                -Constants.Chassi.VELOCIDADE_MAXIMA_MPS,
                Constants.Chassi.VELOCIDADE_MAXIMA_MPS
            ),
            MathUtil.clamp(
                speeds.omegaRadiansPerSecond,
                -Constants.Chassi.VELOCIDADE_ANGULAR_MAXIMA_RAD_S,
                Constants.Chassi.VELOCIDADE_ANGULAR_MAXIMA_RAD_S
            )
        );

        ChassisSpeeds velocidadesDiscretas = ChassisSpeeds.discretize(
            velocidadesLimitadas,
            Constants.Chassi.PERIODO_CONTROLE_SEGUNDOS
        );

        SwerveModuleState[] estadosDesejados;
        if (
            Math.abs(velocidadesDiscretas.vxMetersPerSecond) < 1e-9
            && Math.abs(velocidadesDiscretas.vyMetersPerSecond) < 1e-9
            && Math.abs(velocidadesDiscretas.omegaRadiansPerSecond) < 1e-9
        ) {
            SwerveModuleState[] estadosAtuais = getModuleStates();
            estadosDesejados = new SwerveModuleState[4];
            for (int i = 0; i < 4; i++) {
                estadosDesejados[i] = new SwerveModuleState(
                    0.0,
                    estadosAtuais[i].angle
                );
            }
        } else {
            estadosDesejados = kinematics.toSwerveModuleStates(
                velocidadesDiscretas
            );
        }

        SwerveDriveKinematics.desaturateWheelSpeeds(
            estadosDesejados,
            Constants.Chassi.VELOCIDADE_MAXIMA_MPS
        );

        Logger.recordOutput("SwerveStates/Setpoints", estadosDesejados);
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", velocidadesDiscretas);

        for (int i = 0; i < modules.length; i++) {
            modules[i].runSetpoint(estadosDesejados[i]);
        }

        Logger.recordOutput("SwerveStates/SetpointsOptimized", estadosDesejados);
    }

    /**
     * Pipeline fisicamente limitado usado pelo modo RETO. Alem do discretize,
     * limita aceleracao de roda e taxa de steering antes de enviar aos modulos.
     */
    public void runVelocityAdvanced(ChassisSpeeds speeds) {
        ChassisSpeeds limited = new ChassisSpeeds(
            MathUtil.clamp(
                speeds.vxMetersPerSecond,
                -Constants.Chassi.VELOCIDADE_MAXIMA_MPS,
                Constants.Chassi.VELOCIDADE_MAXIMA_MPS
            ),
            MathUtil.clamp(
                speeds.vyMetersPerSecond,
                -Constants.Chassi.VELOCIDADE_MAXIMA_MPS,
                Constants.Chassi.VELOCIDADE_MAXIMA_MPS
            ),
            MathUtil.clamp(
                speeds.omegaRadiansPerSecond,
                -Constants.Chassi.VELOCIDADE_ANGULAR_MAXIMA_RAD_S,
                Constants.Chassi.VELOCIDADE_ANGULAR_MAXIMA_RAD_S
            )
        );

        ChassisSpeeds discrete = ChassisSpeeds.discretize(
            limited,
            Constants.Chassi.PERIODO_CONTROLE_SEGUNDOS
        );

        SwerveModuleState[] states = advancedSetpointGenerator.generate(
            discrete,
            getModuleStates(),
            kinematics,
            Constants.Chassi.VELOCIDADE_MAXIMA_MPS,
            Constants.ModoReto.ACELERACAO_RODA_MAXIMA_MPS2,
            Constants.ModoReto.VELOCIDADE_STEER_MAXIMA_RAD_S,
            Constants.ModoReto.FATOR_MINIMO_COSINE_COMPENSATION,
            Constants.Chassi.PERIODO_CONTROLE_SEGUNDOS
        );

        Logger.recordOutput("SwerveStates/AdvancedSetpoints", states);
        Logger.recordOutput("SwerveChassisSpeeds/AdvancedSetpoints", discrete);

        for (int i = 0; i < modules.length; i++) {
            modules[i].runPreparedSetpoint(states[i]);
        }
    }

    public void alignModulesForward() {
        // O zero do modulo e definido pelos MagnetOffsets dos CANcoders.
        // FRENTE_GRAUS (-90) pertence ao frame do driver, nao ao steering.
        Rotation2d moduleForward = Rotation2d.kZero;
        for (Module module : modules) {
            module.alignTo(moduleForward);
        }
    }

    public boolean areModulesAlignedForward() {
        Rotation2d moduleForward = Rotation2d.kZero;
        for (Module module : modules) {
            double error = Math.abs(
                MathUtil.angleModulus(
                    module.getAngle().minus(moduleForward).getRadians()
                )
            );
            if (error > Constants.ModoReto.TOLERANCIA_ALINHAMENTO_MODULO_RAD) {
                return false;
            }
        }
        return true;
    }

    public double getMaxModuleAlignmentErrorDeg() {
        Rotation2d moduleForward = Rotation2d.kZero;
        double max = 0.0;
        for (Module module : modules) {
            max = Math.max(
                max,
                Math.abs(Math.toDegrees(
                    MathUtil.angleModulus(
                        module.getAngle().minus(moduleForward).getRadians()
                    )
                ))
            );
        }
        return max;
    }

    public double getTranslationalSpeedMps() {
        ChassisSpeeds speeds = getChassisSpeeds();
        return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    }

    private void runDriveCharacterization(double volts) {
        // SysId de tracao: rodas alinhadas no zero calibrado do steering.
        Rotation2d moduleForward = Rotation2d.kZero;
        for (Module module : modules) {
            module.runDriveCharacterization(volts, moduleForward);
        }
        SmartDashboard.putNumber("SysId/CommandedVolts", volts);
        Logger.recordOutput("Drive/SysId/CommandedVolts", volts);
    }

    private void logSysId(edu.wpi.first.wpilibj.sysid.SysIdRoutineLog log) {
        String[] names = {"front-left", "front-right", "back-left", "back-right"};
        for (int i = 0; i < modules.length; i++) {
            log.motor(names[i])
                .voltage(Volts.of(modules[i].getDriveAppliedVolts()))
                .linearPosition(Meters.of(modules[i].getPositionMeters()))
                .linearVelocity(MetersPerSecond.of(modules[i].getVelocityMetersPerSec()))
                .current(Amps.of(modules[i].getDriveCurrentAmps()));
        }
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction)
            .finallyDo(interrupted -> stop());
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction)
            .finallyDo(interrupted -> stop());
    }

    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    public void zeroHeading() {
        driverGyroOffset = getPose().getRotation();
    }

    public Rotation2d getRotation() {
        return getPose().getRotation().minus(driverGyroOffset);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public Rotation2d getRawGyroRotation() {
        return rawGyroRotation;
    }

    public double getYawVelocityRadPerSec() {
        return gyroInputs.yawVelocityRadPerSec;
    }

    public Rotation2d adicionarMedicaoVisao(
        Pose2d poseVisao,
        double timestampSegundos,
        double desvioPadraoXMetros,
        double desvioPadraoYMetros,
        double desvioPadraoThetaRad
    ) {
        Rotation2d headingAntes = getPose().getRotation();

        poseEstimator.addVisionMeasurement(
            poseVisao,
            timestampSegundos,
            VecBuilder.fill(
                desvioPadraoXMetros,
                desvioPadraoYMetros,
                desvioPadraoThetaRad
            )
        );

        Rotation2d headingDepois = getPose().getRotation();
        Rotation2d correcaoHeading = headingDepois.minus(headingAntes);

        Logger.recordOutput("Odometry/VisionPose", poseVisao);
        Logger.recordOutput(
            "Odometry/VisionHeadingCorrectionDeg",
            correcaoHeading.getDegrees()
        );

        return correcaoHeading;
    }

    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    public static Translation2d[] getModuleTranslations() {
        return new Translation2d[] {
            new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
            new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
            new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
            new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
        };
    }

    private void atualizarOdometria() {
        double[] timestamps = modules[0].getOdometryTimestamps();

        for (int amostra = 0; amostra < timestamps.length; amostra++) {
            SwerveModulePosition[] posicoes = new SwerveModulePosition[4];
            SwerveModulePosition[] deltas = new SwerveModulePosition[4];

            for (int modulo = 0; modulo < modules.length; modulo++) {
                SwerveModulePosition[] posicoesModulo = modules[modulo]
                    .getOdometryPositions();

                if (amostra >= posicoesModulo.length) {
                    return;
                }

                posicoes[modulo] = posicoesModulo[amostra];
                deltas[modulo] = new SwerveModulePosition(
                    posicoes[modulo].distanceMeters
                        - lastModulePositions[modulo].distanceMeters,
                    posicoes[modulo].angle
                );
                lastModulePositions[modulo] = posicoes[modulo];
            }

            if (
                gyroInputs.connected
                && amostra < gyroInputs.odometryYawPositions.length
            ) {
                rawGyroRotation = gyroInputs.odometryYawPositions[amostra];
            } else {
                Twist2d twist = kinematics.toTwist2d(deltas);
                rawGyroRotation = rawGyroRotation.plus(
                    new Rotation2d(twist.dtheta)
                );
            }

            poseEstimator.updateWithTime(
                timestamps[amostra],
                rawGyroRotation,
                posicoes
            );
        }
    }

    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] estados = new SwerveModuleState[4];
        for (int i = 0; i < modules.length; i++) {
            estados[i] = modules[i].getState();
        }
        return estados;
    }
}
