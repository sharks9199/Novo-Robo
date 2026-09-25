package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
import frc.robot.subsystems.drive.control.ModoRetoController;
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
    private final ModoRetoController modoReto = new ModoRetoController();
    private final Elevator elevador = criarElevador();
    private final Vision vision;

    public RobotContainer() {
        vision = new Vision(
            new VisionIOLimelight(Constants.Vision.NOME_LIMELIGHT),
            chassi::getHeading,
            chassi::getYawVelocityRadPerSec,
            this::aplicarMedicaoVisao
        );

        configurarComandosPadrao();
        configurarBotoes();
        configurarSysIdElastic();
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

    private void configurarComandosPadrao() {
        chassi.setDefaultCommand(
            DriveCommands.joystickDrive(
                chassi,
                headingController,
                modoReto,
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

        // BOTAO PRINCIPAL DO MODO RETO:
        // A alterna todo o pacote avancado ON/OFF.
        controleDriver.a().onTrue(
            Commands.runOnce(this::alternarModoReto)
        );

        // O A agora pertence ao Modo RETO. O alinhamento da Limelight foi preservado no X.
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

    /**
     * Funcao principal do botao A.
     * Liga/desliga o Modo RETO inteiro. Ao ligar, a orientacao atual vira 0°
     * e o HeadingController passa a trabalhar nesse novo referencial.
     */
    private void alternarModoReto() {
        boolean ativou = modoReto.toggle();

        if (ativou) {
            zerarReferenciaDriver();
        } else {
            headingController.desativar();
        }

        SmartDashboard.putBoolean("ModoReto/Enabled", ativou);
    }

    private void zerarReferenciaDriver() {
        chassi.zerarOrientacaO();
        if (modoReto.estaLigado()) {
            headingController.definirHeading(
                Rotation2d.kZero,
                chassi.getRotation(),
                chassi.getYawVelocityRadPerSec()
            );
        }
    }

    private void configurarSysIdElastic() {
        SmartDashboard.putData(
            "SysId/Quasistatic Forward",
            prepararSysId().andThen(
                chassi.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
            )
        );
        SmartDashboard.putData(
            "SysId/Quasistatic Reverse",
            prepararSysId().andThen(
                chassi.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
            )
        );
        SmartDashboard.putData(
            "SysId/Dynamic Forward",
            prepararSysId().andThen(
                chassi.sysIdDynamic(SysIdRoutine.Direction.kForward)
            )
        );
        SmartDashboard.putData(
            "SysId/Dynamic Reverse",
            prepararSysId().andThen(
                chassi.sysIdDynamic(SysIdRoutine.Direction.kReverse)
            )
        );
    }

    private Command prepararSysId() {
        return Commands.runOnce(() -> {
            modoReto.desativar();
            headingController.desativar();
        });
    }

    public Command getAutonomousCommand() {
        return Commands.none().withName("Sem autônomo");
    }
}
