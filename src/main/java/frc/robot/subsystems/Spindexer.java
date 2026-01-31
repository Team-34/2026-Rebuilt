package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spindexer extends SubsystemBase {
    public TalonFX spindexerMotor = new TalonFX(55);

    public Command spin() {
        return runOnce(() -> {
            spindexerMotor.set(0.5);
        });
    }

    public Command spinStop() {
        return runOnce(() -> {
            spindexerMotor.set(0);
        });
    }

    public Command spinReverse() {
        return runOnce(() -> {
            spindexerMotor.set(-0.5);
        });
    }
}
