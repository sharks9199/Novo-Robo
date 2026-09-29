package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AlinharComLimelight;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOComp;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.control.HeadingController;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.Preset;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelight;

public class RobotContainer {
    private final CommandXboxController controleDriver =
        new CommandXboxController(Constants.Controles.PORTA_DRIVER);

    private final CommandXboxController controleOperador =
        new CommandXboxController(Constants.Controles.PORTA_OPERADOR);

    private final Drive chassi = criarChassi();
    private final HeadingController headingController = new HeadingController();
    private final Elevator elevador = criarElevador();
    private final Vision vision;
    private final SendableChooser<Command> seletorAutonomo;

    public RobotContainer() {
        vision = new Vision(
            new VisionIOLimelight(Constants.Vision.NOME_LIMELIGHT),
            chassi::getHeading,
            chassi::getYawVelocityRadPerSec,
            this::aplicarMedicaoVisao
        );

        registrarComandosNomeados();
        // Precisa vir depois do AutoBuilder (configurado no Drive) e dos comandos nomeados.
        seletorAutonomo = criarSeletorAutonomo();
        SmartDashboard.putData("Autonomo/Seletor", seletorAutonomo);

        configurarComandosPadrao();
        configurarBotoes();
    }

    private Drive criarChassi() {
        return switch (Constants.MODO_ATUAL) {
            case REAL -> new Drive(
                new GyroIOPigeon2(),
                new ModuleIOComp(TunerConstants.FrontLeft),
                new ModuleIOComp(TunerConstants.FrontRight),
                new ModuleIOComp(TunerConstants.BackLeft),
                new ModuleIOComp(TunerConstants.BackRight)
            );
            case SIM -> new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight)
            );
            case REPLAY -> new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {}
            );
        };
    }

    private Elevator criarElevador() {
        return Constants.MODO_ATUAL == Constants.Mode.REAL
            ? new Elevator(new ElevatorIOReal())
            : new Elevator(new ElevatorIOSim());
    }

    private void aplicarMedicaoVisao(Vision.MedicaoVisao medicao) {
        Rotation2d correcaoHeading = chassi.adicionarMedicaoVisao(
            medicao.pose(),
            medicao.timestampSegundos(),
            medicao.desvioPadraoXMetros(),
            medicao.desvioPadraoYMetros(),
            medicao.desvioPadraoThetaRad()
        );

        headingController.aplicarCorrecaoReferencia(correcaoHeading);
    }

    private SendableChooser<Command> criarSeletorAutonomo() {
        try {
            return AutoBuilder.buildAutoChooser();
        } catch (Exception e) {
            DriverStation.reportError(
                "Falha ao criar o seletor de autonomo: " + e.getMessage(),
                e.getStackTrace()
            );
            SendableChooser<Command> vazio = new SendableChooser<>();
            vazio.setDefaultOption("Nenhum", Commands.none());
            return vazio;
        }
    }

    /** Comandos que podem ser usados como "Named Command" nos autos do PathPlanner. */
    private void registrarComandosNomeados() {
        NamedCommands.registerCommand(
            "ElevadorH0",
            Commands.runOnce(() -> elevador.irParaPreset(Preset.H0), elevador)
        );
        NamedCommands.registerCommand(
            "ElevadorH1",
            Commands.runOnce(() -> elevador.irParaPreset(Preset.H1), elevador)
        );
        NamedCommands.registerCommand(
            "ElevadorH2",
            Commands.runOnce(() -> elevador.irParaPreset(Preset.H2), elevador)
        );
        NamedCommands.registerCommand(
            "ElevadorH3",
            Commands.runOnce(() -> elevador.irParaPreset(Preset.H3), elevador)
        );
        NamedCommands.registerCommand(
            "ElevadorParar",
            Commands.runOnce(elevador::cancelarTudo, elevador)
        );
    }

    private void configurarComandosPadrao() {
        chassi.setDefaultCommand(
            DriveCommands.joystickDrive(
                chassi,
                headingController,
                () -> -controleDriver.getLeftY(),
                () -> -controleDriver.getLeftX(),
                () -> -controleDriver.getRightX()
            )
        );

        elevador.setDefaultCommand(
            Commands.run(
                () -> elevador.atualizarEntradaOperador(
                    controleOperador.getHID().getPOV(),
                    controleOperador.getRightTriggerAxis()
                ),
                elevador
            )
        );
    }

    private void configurarBotoes() {
        controleOperador.rightBumper().onTrue(
            Commands.runOnce(
                elevador::cancelarTudo,
                elevador
            )
        );

        controleDriver.x().whileTrue(
            new AlinharComLimelight(vision, chassi)
        );

        controleOperador.leftBumper().onTrue(
            Commands.runOnce(
                elevador::zerarEncodersAltura,
                elevador
            )
        );

        controleOperador.a().onTrue(
            Commands.runOnce(
                () -> elevador.irParaPreset(Preset.H0),
                elevador
            )
        );

        controleOperador.x().onTrue(
            Commands.runOnce(
                () -> elevador.irParaPreset(Preset.H1),
                elevador
            )
        );

        controleOperador.y().onTrue(
            Commands.runOnce(
                () -> elevador.irParaPreset(Preset.H2),
                elevador
            )
        );

        controleOperador.b().onTrue(
            Commands.runOnce(
                () -> elevador.irParaPreset(Preset.H3),
                elevador
            )
        );

        controleDriver.start().onTrue(
            Commands.runOnce(this::zerarReferenciaDriver)
        );
    }

    private void zerarReferenciaDriver() {
        chassi.zerarOrientacaO();
        // O heading é recapturado sozinho na próxima translação.
        headingController.desativar();
    }

    public Command getAutonomousCommand() {
        Command selecionado = seletorAutonomo.getSelected();
        return selecionado != null ? selecionado : Commands.none();
    }
}
