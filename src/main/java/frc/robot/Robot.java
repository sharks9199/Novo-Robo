  // Copyright (c) FIRST and other WPILib contributors.
  // Open Source Software; you can modify and/or share it under the terms of
  // the WPILib BSD license file in the root directory of this project.

  package frc.robot;

  import edu.wpi.first.wpilibj2.command.Command;
  import edu.wpi.first.wpilibj2.command.CommandScheduler;
  import org.littletonrobotics.junction.LogFileUtil;
  import org.littletonrobotics.junction.Logger;
  import org.littletonrobotics.junction.networktables.NT4Publisher;
  import org.littletonrobotics.junction.wpilog.WPILOGReader;
  import org.littletonrobotics.junction.wpilog.WPILOGWriter;
  import org.littletonrobotics.junction.LoggedRobot;

  /**
   * Main robot class. Configures logging, owns the {@link RobotContainer} and drives the command
   * scheduler for every robot mode.
   */
  public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    /** Starts AdvantageKit logging and builds the {@link RobotContainer}. */
    public Robot() {
      if (Constants.MODO_ATUAL == Constants.Mode.REPLAY) {
        // Replay a log as fast as possible and write the re-processed output next to it.


        //Logger.setUseTiming(false);
        setUseTiming(false); //aí processa os ciclos do replay tão rápido quanto o programa e o computador permitem.
        //                               ex: um log de 2 minutos pode ser processado em muito menos tempo, dependendo da carga de processamento.




        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        Logger.addDataReceiver(new NT4Publisher());
      } else {
        // Publish data to NetworkTables and, in simulation, also write a .wpilog file.
        Logger.addDataReceiver(new NT4Publisher());
        if (isSimulation()) {
          Logger.addDataReceiver(new WPILOGWriter());
        }
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

    /** Releases the heading controller and path target so teleop starts from a clean state. */
    @Override
    public void autonomousExit() {
      m_robotContainer.aoFinalizarAutonomo();
    }

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
