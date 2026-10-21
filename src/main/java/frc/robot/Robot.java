// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * Main robot class. Configures logging, owns the {@link RobotContainer} and drives the command
 * scheduler for every robot mode.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  /** Starts AdvantageKit logging and builds the {@link RobotContainer}. */
  public Robot() {
    // Publish data to NetworkTables and, in simulation, also write a .wpilog file.
    Logger.addDataReceiver(new NT4Publisher());
    if (isSimulation()) {
      Logger.addDataReceiver(new WPILOGWriter());
    }
    Logger.start();

    m_robotContainer = new RobotContainer();
  }

  // ---------------------------------------------------------------------------
  // Periodic (all modes)
  // ---------------------------------------------------------------------------

  /** Runs the command scheduler. Must be called every loop, regardless of mode. */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  // ---------------------------------------------------------------------------
  // Disabled
  // ---------------------------------------------------------------------------

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  // ---------------------------------------------------------------------------
  // Autonomous
  // ---------------------------------------------------------------------------

  /** Schedules the autonomous command selected on the dashboard, if any. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  // ---------------------------------------------------------------------------
  // Teleoperated
  // ---------------------------------------------------------------------------

  /** Cancels the autonomous command so that the driver takes over immediately. */
  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  // ---------------------------------------------------------------------------
  // Test
  // ---------------------------------------------------------------------------

  /** Cancels all running commands when entering test mode. */
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
