package frc.robot.subsystems.drive.control;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/** Estado global do modo RETO acionado por toggle no botao A. */
public final class ModoRetoController {
    public enum Estado {
        DESLIGADO,
        ATIVO
    }

    private Estado estado = Estado.DESLIGADO;

    public ModoRetoController() {
        publicarDashboard();
    }

    public boolean toggle() {
        if (estaLigado()) {
            desativar();
            return false;
        }
        ativar();
        return true;
    }

    public void ativar() {
        estado = Estado.ATIVO;
        publicarDashboard();
    }

    public void desativar() {
        estado = Estado.DESLIGADO;
        publicarDashboard();
    }

    public boolean estaLigado() {
        return estado != Estado.DESLIGADO;
    }


    public boolean estaAtivo() {
        return estado == Estado.ATIVO;
    }

    public Estado getEstado() {
        return estado;
    }

    public boolean deveSnaparZero(Rotation2d headingRelativo) {
        return Math.abs(headingRelativo.getRadians()) <= Constants.ModoReto.JANELA_SNAP_ZERO_RAD;
    }

    public void publicarDashboard() {
        SmartDashboard.putBoolean("ModoReto/Enabled", estaLigado());
        SmartDashboard.putString("ModoReto/Estado", estado.name());
    }
}
