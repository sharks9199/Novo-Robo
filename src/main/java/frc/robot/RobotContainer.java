package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.swerve.SwerveCommands;
import frc.robot.subsystems.swerve.Drive;
import frc.robot.subsystems.swerve.control.HeadingController;

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

  private final CommandXboxController controleDoDriver =
      new CommandXboxController(Constants.Controles.PORTA_DRIVER);

  // ---------------------------------------------------------------------------
  // Dashboard
  // ---------------------------------------------------------------------------

  private final SendableChooser<Command> meuSeletorAuto;

  /** Builds the autonomous chooser, publishes it to the dashboard and wires up the controls. */
  public RobotContainer() {
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
            () -> -controleDoDriver.getLeftY(),
            () -> -controleDoDriver.getLeftX(),
            () -> -controleDoDriver.getRightX()));
  }

  /** Binds driver controller buttons to commands. */
  public void configureButtons() {
    // Start: reset the field-relative heading.
    controleDoDriver.start().onTrue(Commands.runOnce(this::zerarGuinada));



               /*ControleDoDriver.a().onTrue(
                      Commands.runOnce(null, null)
                       )*/

                       
  }

  // ---------------------------------------------------------------------------
  // Autonomous
  // ---------------------------------------------------------------------------

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

  /** Resets the field-relative heading and disables the heading-hold controller. */
  public void zerarGuinada() {
    swerve.zerarOrientacaO();
    headingController.desativar();
  }
}
