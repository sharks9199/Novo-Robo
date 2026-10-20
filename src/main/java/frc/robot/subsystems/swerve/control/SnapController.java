package frc.robot.subsystems.swerve.control;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

public final class SnapController {
    private enum SnapAxis {
        NONE,
        LONGITUDINAL,
        LATERAL
    }

    private SnapAxis snapAxis = SnapAxis.NONE;

    public Translation2d process(double longitudinal, double lateral) {
        atualizarSnap(longitudinal, lateral);

        double x = longitudinal;
        double y = lateral;

        if (snapAxis == SnapAxis.LONGITUDINAL) {
            y = 0.0;
        } else if (snapAxis == SnapAxis.LATERAL) {
            x = 0.0;
        }

        double rawMagnitude = Math.hypot(x, y);
        double magnitude = Math.min(rawMagnitude, 1.0);
        if (magnitude <= Constants.ControlesDrive.DEADBAND_RADIAL) {
            return Translation2d.kZero;
        }

        double magnitudeSemDeadband = MathUtil.applyDeadband(
            magnitude,
            Constants.ControlesDrive.DEADBAND_RADIAL
        );
        double magnitudeModelada = Math.pow(
            magnitudeSemDeadband,
            Constants.ControlesDrive.EXPO_TRANSLACAO
        );

        return new Translation2d(x / rawMagnitude, y / rawMagnitude)
            .times(magnitudeModelada);
    }

    private void atualizarSnap(double longitudinal, double lateral) {
        double absLongitudinal = Math.abs(longitudinal);
        double absLateral = Math.abs(lateral);

        if (
            Math.max(absLongitudinal, absLateral)
                < Constants.ControlesDrive.EIXO_PRINCIPAL_MINIMO_SNAP
        ) {
            snapAxis = SnapAxis.NONE;
            return;
        }

        if (snapAxis == SnapAxis.LONGITUDINAL) {
            if (absLateral > Constants.ControlesDrive.EIXO_SECUNDARIO_SAIR_SNAP) {
                snapAxis = SnapAxis.NONE;
            }
            return;
        }

        if (snapAxis == SnapAxis.LATERAL) {
            if (
                absLongitudinal
                    > Constants.ControlesDrive.EIXO_SECUNDARIO_SAIR_SNAP
            ) {
                snapAxis = SnapAxis.NONE;
            }
            return;
        }

        if (
            absLongitudinal >= absLateral
            && absLateral <= Constants.ControlesDrive.EIXO_SECUNDARIO_ENTRAR_SNAP
        ) {
            snapAxis = SnapAxis.LONGITUDINAL;
        } else if (
            absLateral > absLongitudinal
            && absLongitudinal
                <= Constants.ControlesDrive.EIXO_SECUNDARIO_ENTRAR_SNAP
        ) {
            snapAxis = SnapAxis.LATERAL;
        }
    }

    public boolean isSnappingLongitudinal() {
        return snapAxis == SnapAxis.LONGITUDINAL;
    }

    public boolean isSnappingLateral() {
        return snapAxis == SnapAxis.LATERAL;
    }

    public void reset() {
        snapAxis = SnapAxis.NONE;
    }
}
