package frc.robot.subsystems.drive;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OffsetsConstants;
import frc.robot.generated.TunerConstants;

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

    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    public void zeroHeading() {
        driverGyroOffset = getPose().getRotation();
    }

    public Rotation2d getRotation() {
        return getPose().getRotation().minus(driverGyroOffset);
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
