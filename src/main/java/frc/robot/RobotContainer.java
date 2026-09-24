package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AlinharComLimelight;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.SetStraightMovement;
import frc.robot.commands.SeguirBolinha;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOReal;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOComp;
import frc.robot.subsystems.drive.ModuleIOSim;
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
    private final Elevator elevador = criarElevador();
    private final Vision vision = new Vision(
        new VisionIOLimelight("limelight-sharks")
    );

    public RobotContainer() {
        configurarComandosPadrao();
        configurarBotoes();

        Commands.run(() -> {
            LimelightHelpers.PoseEstimate estimativa = vision.pegarPoseMegaTag2();
            if (estimativa.tagCount > 0) {
                chassi.adicionarMedicaoVision(
                    estimativa.pose,
                    estimativa.timestampSeconds
                );
            }
        }).schedule();
    }

    private Drive criarChassi() {
        return switch (Constants.MODO_ATUAL) {
            case REAL -> new Drive(
                new GyroIOReal(),
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

    private void configurarComandosPadrao() {
        chassi.setDefaultCommand(
            DriveCommands.joystickDrive(
                chassi,
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

        controleDriver.a().whileTrue(
            new AlinharComLimelight(vision, chassi)
        );

        controleDriver.b().whileTrue(
            new SeguirBolinha(vision, chassi)
        );

        controleDriver.rightTrigger().whileTrue(
            new SetStraightMovement(chassi)
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
            Commands.runOnce(
                chassi::zerarOrientacaO,
                chassi
            )
        );
    }

    public Command getAutonomousCommand() {
        return Commands.none().withName("Sem autônomo");
    }
}
