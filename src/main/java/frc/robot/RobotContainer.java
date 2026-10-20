package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.swerve.SwerveCommands;
import frc.robot.subsystems.swerve.Drive;
import frc.robot.subsystems.swerve.control.HeadingController;

public class RobotContainer {
    private final Drive swerve = Drive.criarSwerve();
    private final HeadingController headingController = new HeadingController();

    private final CommandXboxController controleDoDriver = new CommandXboxController(Constants.Controles.PORTA_DRIVER);
    private final SendableChooser<Command> meuSeletorAuto;

    public RobotContainer() {
      
      // =========== Colocando o Autônomo no Elastic ===========
      meuSeletorAuto = doAutoSelector();
      SmartDashboard.putData("Autonomo/Seletor", meuSeletorAuto);

      configureBindings();
      configureButtons();
    }

    private void configureBindings(){
      swerve.setDefaultCommand(
        SwerveCommands.movimentacaoDrive(
          swerve,
          headingController,
          () -> -controleDoDriver.getLeftY(),
          () -> -controleDoDriver.getLeftX(),
          () -> -controleDoDriver.getRightX()
        )
      );
    }

    public void configureButtons(){

      controleDoDriver.start().onTrue(
        Commands.runOnce(this::zerarGuinada));


      /*ControleDoDriver.a().onTrue(
        Commands.runOnce(null, null)
      )*/

    }

    private SendableChooser<Command> doAutoSelector(){
      // ===== Tentativa -> sout =======
      try{
        return AutoBuilder.buildAutoChooser();
      } catch (Exception e){
        DriverStation.reportError("Erro ao criar o Autônomo: " + e.getMessage(), e.getStackTrace());

        SendableChooser<Command> autonomoVazio = new SendableChooser<>();
        autonomoVazio.setDefaultOption("Nenhum Autônomo", Commands.none());
        return autonomoVazio;
      }

    }

    public void zerarGuinada(){
      swerve.zerarOrientacaO();
      headingController.desativar();
    }
    

    public Command getAutonomousCommand(){
      Command autoSelecionado = meuSeletorAuto.getSelected();
      return autoSelecionado != null ? autoSelecionado : Commands.none(); //Operador ternário bem útil
    }

        
}
  