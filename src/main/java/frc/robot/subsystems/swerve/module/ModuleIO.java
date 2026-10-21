package frc.robot.subsystems.swerve.module;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Hardware abstraction for a single swerve module (drive motor, turn motor and encoder). */
public interface ModuleIO {

  /** Module inputs logged by AdvantageKit. */
  @AutoLog
  public static class ModuleIOInputs {
    // ----- Drive motor -----
    public boolean driveConnected = false;
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveCurrentAmps = 0.0;

    // ----- Turn motor and encoder -----
    public boolean turnConnected = false;
    public boolean turnEncoderConnected = false;
    public Rotation2d turnAbsolutePosition = new Rotation2d();
    public Rotation2d turnPosition = new Rotation2d();
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnCurrentAmps = 0.0;

    // ----- High-frequency odometry samples -----
    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsRad = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
  }

  /** Refreshes the given inputs with the latest module data. */
  public default void updateInputs(ModuleIOInputs inputs) {}

  /** Runs the drive motor in open loop (volts or amps, depending on the configured output). */
  public default void setDriveOpenLoop(double output) {}

  /** Runs the turn motor in open loop (volts or amps, depending on the configured output). */
  public default void setTurnOpenLoop(double output) {}

  /** Runs the drive motor in closed-loop velocity control, in rad/s. */
  public default void setDriveVelocity(double velocityRadPerSec) {}

  /** Runs the turn motor in closed-loop position control. */
  public default void setTurnPosition(Rotation2d rotation) {}
}
