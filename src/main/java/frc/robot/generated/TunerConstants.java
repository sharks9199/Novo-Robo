package frc.robot.generated;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants;

/**
 * Entry point for the swerve module constants expected by the IO layer and by Phoenix Tuner X.
 *
 * <p>This class no longer holds its own copy of the configuration. Every value is read from {@link
 * Constants.Swerve}, which is the single source of truth. Before this change the two classes held
 * different values for the same hardware, which made the odometry and the motor configuration
 * disagree. The values that used to live here were:
 *
 * <ul>
 *   <li>Speed at 12 V: 4.5 m/s (now {@link Constants.Swerve#VELOCIDADE_FISICA_A_12V}, 4.20 m/s)
 *   <li>Wheel radius: 0.048 m (now 0.0964 / 2 m)
 *   <li>Drive inertia: 0.025 kg*m^2 (now 0.020) and drive friction: 0.2 V (now 0.25)
 * </ul>
 *
 * <p>If Tuner X regenerates this file, re-apply this delegation or copy the new values into {@link
 * Constants.Swerve}.
 */
public final class TunerConstants {

  /** Pigeon 2 configuration, including its mounting orientation. */
  public static final Pigeon2Configuration CONFIGURACAO_PIGEON =
      Constants.Swerve.CONFIGURACAO_PIGEON;

  /** Approximate physical speed at 12 V, used by the module model. */
  public static final LinearVelocity VELOCIDADE_FISICA_A_12V =
      Constants.Swerve.VELOCIDADE_FISICA_A_12V;

  /** Drivetrain-level constants (CAN bus name, Pigeon 2 ID and configuration). */
  public static final SwerveDrivetrainConstants DrivetrainConstants = Constants.Swerve.DRIVETRAIN;

  /** CAN bus the drivetrain devices are connected to. */
  public static final CANBus CAN_BUS = Constants.Swerve.CAN_BUS;

  /** Front-left module (steer 2, drive 1, encoder 9). */
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontLeft = Constants.Swerve.FRONT_LEFT;

  /** Front-right module (steer 4, drive 3, encoder 10). */
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontRight = Constants.Swerve.FRONT_RIGHT;

  /** Back-left module (steer 6, drive 5, encoder 11). */
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackLeft = Constants.Swerve.BACK_LEFT;

  /** Back-right module (steer 8, drive 7, encoder 12). */
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackRight = Constants.Swerve.BACK_RIGHT;

  private TunerConstants() {}
}
