package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.swerve.SwerveCommands;
import frc.robot.subsystems.swerve.Drive;
import frc.robot.subsystems.swerve.control.HeadingController;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOReal;
import frc.robot.subsystems.vision.VisionIOSim;

/**
 * Declares the robot's structure: subsystems, controllers, button bindings and the autonomous
 * chooser. Very little robot logic should live here; it belongs in subsystems and commands.
 */
public class RobotContainer {

  // ---------------------------------------------------------------------------
  // Subsystems and controllers
  // ---------------------------------------------------------------------------

  private final Drive swerve = Drive.criarSwerve();
  private final HeadingController headingController = new HeadingController();

  //Parte 9   
  /** AprilTag localization; it fuses its measurements into the swerve pose estimator. */
  @SuppressWarnings("unused")
  private final Vision vision =
      new Vision(
          swerve,
          switch (Constants.MODO_ATUAL) {
            case REAL -> new VisionIOReal();
            case SIM -> new VisionIOSim(swerve::getPose);
            case REPLAY -> new VisionIO() {};
          });

          

  private static final int PORTA_DRIVER = 0;
  private static final int PORTA_OPERADOR = 1;

  //dois controle usando a mesma porta
  private final CommandPS5Controller controleDoDriverPS5 =
      new CommandPS5Controller(PORTA_DRIVER);

  private final CommandXboxController controleDoDriverXbox =
      new CommandXboxController(PORTA_DRIVER);

  @SuppressWarnings("unused")
  private final CommandXboxController controleOperador =
    new CommandXboxController(PORTA_OPERADOR);

  private boolean usandoPS5() {
      return DriverStation.isJoystickConnected(PORTA_DRIVER)
          && !DriverStation.getJoystickIsXbox(PORTA_DRIVER);
  }
  private boolean usandoXbox() {
      return DriverStation.isJoystickConnected(PORTA_DRIVER)
          && DriverStation.getJoystickIsXbox(PORTA_DRIVER);
  }

      
    

  // ---------------------------------------------------------------------------
  // Dashboard
  // ---------------------------------------------------------------------------

  private final SendableChooser<Command> meuSeletorAuto;

  /** Builds the autonomous chooser, publishes it to the dashboard and wires up the controls. */
  public RobotContainer() {
    // The same heading controller is shared by teleop and autonomous path following.
    swerve.usarHeadingControllerNoAutonomo(headingController);

    // Keep the heading goal consistent when vision corrects the estimated rotation.
    swerve.setHeadingCorrectionListener(headingController::aplicarCorrecaoReferencia);

    // Named commands must be registered BEFORE the autonomous chooser is built.
    configureNamedCommands();

    // Publish the autonomous chooser to the dashboard (Elastic).
    meuSeletorAuto = doAutoSelector();
    SmartDashboard.putData("Autonomo/Seletor", meuSeletorAuto);

    configureBindings();
    configureButtons();
    
  }

  // ---------------------------------------------------------------------------
  // Bindings
  // ---------------------------------------------------------------------------

  /** Sets the default drive command (field-relative teleop with heading hold). */

private void configureBindings() {
    swerve.setDefaultCommand(
        SwerveCommands.movimentacaoDrive(
            swerve,
            headingController,

            () -> usandoPS5()
                ? -controleDoDriverPS5.getLeftY()
                : usandoXbox()
                    ? -controleDoDriverXbox.getLeftY()
                    : 0.0,

            () -> usandoPS5()
                ? -controleDoDriverPS5.getLeftX()
                : usandoXbox()
                    ? -controleDoDriverXbox.getLeftX()
                    : 0.0,

            () -> usandoPS5()
                ? -controleDoDriverPS5.getRightX()
                : usandoXbox()
                    ? -controleDoDriverXbox.getRightX()
                    : 0.0
        )
    );
}


  /** Binds driver controller buttons to commands. */

public void configureButtons() {

    new Trigger(this::usandoPS5)
        .and(controleDoDriverPS5.options())
        .onTrue(Commands.runOnce(this::zerarGuinada));

    new Trigger(this::usandoXbox)
        .and(controleDoDriverXbox.start())
        .onTrue(Commands.runOnce(this::zerarGuinada));

    new Trigger(this::usandoPS5)
        .and(controleDoDriverPS5.cross())
        .onTrue(Commands.runOnce(swerve::alternarModoLento));

    new Trigger(this::usandoXbox)
        .and(controleDoDriverXbox.a())
        .onTrue(Commands.runOnce(swerve::alternarModoLento));
}   


  // ---------------------------------------------------------------------------
  // Autonomous
  // ---------------------------------------------------------------------------

  /**
   * Registers commands that PathPlanner paths and autos can trigger by name (event markers, named
   * commands). Register them here, before {@link #doAutoSelector()} runs.
   */
  private void configureNamedCommands() {
    // Example: NamedCommands.registerCommand("Shoot", shooter.shootCommand());
  }

  /**
   * Builds the PathPlanner autonomous chooser.
   *
   * <p>If PathPlanner fails to load, the error is reported to the Driver Station and a chooser
   * containing only a "do nothing" routine is returned, so the robot code never crashes at startup.
   *
   * @return the autonomous chooser, never {@code null}
   */
  private SendableChooser<Command> doAutoSelector() {
    try {
      return AutoBuilder.buildAutoChooser();
    } catch (Exception e) {
      DriverStation.reportError("Erro ao criar o Autônomo: " + e.getMessage(), e.getStackTrace());

      SendableChooser<Command> autonomoVazio = new SendableChooser<>();
      autonomoVazio.setDefaultOption("Nenhum Autônomo", Commands.none());
      return autonomoVazio;
    }
  }

  /**
   * Returns the command to run in autonomous.
   *
   * @return the selected autonomous command, or {@link Commands#none()} if nothing is selected
   */
  public Command getAutonomousCommand() {
    Command autoSelecionado = meuSeletorAuto.getSelected();
    return autoSelecionado != null ? autoSelecionado : Commands.none();
  }

  // ---------------------------------------------------------------------------
  // Actions

  // ---------------------------------------------------------------------------

  /** Called when autonomous ends: drops the path heading target and heading-hold goal. */
  public void aoFinalizarAutonomo() {
    swerve.limparAlvoAutonomo();
    headingController.desativar();
  }

  /** Resets the field-relative heading and disables the heading-hold controller. */
  public void zerarGuinada() {
    swerve.zerarOrientacaO();
    headingController.desativar();
  }
}
