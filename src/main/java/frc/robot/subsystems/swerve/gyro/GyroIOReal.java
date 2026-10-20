package frc.robot.subsystems.swerve.gyro;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import frc.robot.Constants;
import frc.robot.subsystems.swerve.Drive;
import frc.robot.subsystems.swerve.SwerveOdometry;

import java.util.Queue;

public class GyroIOReal implements GyroIO {

    private final Pigeon2 pigeon = new Pigeon2(
            Constants.Swerve.DRIVETRAIN.Pigeon2Id,
            Constants.Swerve.CAN_BUS);

    private final StatusSignal<Angle> yaw = pigeon.getYaw();
    private final StatusSignal<AngularVelocity> yawVelocity =
            pigeon.getAngularVelocityZWorld();

    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;

    public GyroIOReal() {
        pigeon.getConfigurator().apply(Constants.Swerve.CONFIGURACAO_PIGEON);

        pigeon.getConfigurator().setYaw(0.0);

        yaw.setUpdateFrequency(Drive.ODOMETRY_FREQUENCY);
        yawVelocity.setUpdateFrequency(50.0);

        pigeon.optimizeBusUtilization();

        yawTimestampQueue =
                SwerveOdometry.getInstance().makeTimestampQueue();

        yawPositionQueue =
                SwerveOdometry.getInstance()
                        .registerSignal(pigeon.getYaw());
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected =
                BaseStatusSignal.refreshAll(yaw, yawVelocity)
                        .equals(StatusCode.OK);

        inputs.yawPosition =
                Rotation2d.fromDegrees(yaw.getValueAsDouble());

        inputs.yawVelocityRadPerSec =
                Units.degreesToRadians(
                        yawVelocity.getValueAsDouble());

        inputs.odometryYawTimestamps =
                yawTimestampQueue.stream()
                        .mapToDouble(value -> value)
                        .toArray();

        inputs.odometryYawPositions =
                yawPositionQueue.stream()
                        .map(Rotation2d::fromDegrees)
                        .toArray(Rotation2d[]::new);

        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }
}