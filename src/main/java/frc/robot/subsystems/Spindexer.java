package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spindexer extends SubsystemBase {
    public TalonFX m_spindexerMotor = new TalonFX(61);

    public Command Spin() {
        return runOnce(() -> {
            m_spindexerMotor.set(0.5);
        });
    }

    public Command Spinstop() {
        return runOnce(() -> {
            m_spindexerMotor.set(0);
        });
    }

    public Command Spinreverse() {
        return runOnce(() -> {
            m_spindexerMotor.set(-0.5);
        });
    }
}
