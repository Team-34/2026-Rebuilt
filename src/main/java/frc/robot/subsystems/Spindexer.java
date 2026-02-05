package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spindexer extends SubsystemBase {
    public final TalonFX Motor = new TalonFX(55);

    private final DutyCycleOut control = new DutyCycleOut(0);

    public Command spin() {
        return runOnce(() -> {
            Motor.setControl(this.control.withOutput(0.5));
        });
    }

    public Command stop() {
        return runOnce(() -> {
            Motor.setControl(this.control.withOutput(0));
        });
    }

    public Command spinReverse() {
        return runOnce(() -> {
            Motor.setControl(this.control.withOutput(-0.5));
        });
    }
}
