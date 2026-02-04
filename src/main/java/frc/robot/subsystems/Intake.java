package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Intake extends SubsystemBase {
  
    private final DoubleSolenoid piston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);

    public final TalonFXS motor = new TalonFXS(60);
    public final DutyCycleOut motorControl = new DutyCycleOut(0);

    public boolean isDeployed() {
        return piston.get() == DoubleSolenoid.Value.kForward;
    }

    public boolean isRetracted() {
        return piston.get() == DoubleSolenoid.Value.kReverse;
    }

    public Command runIn() {
        return runOnce(() -> {
            if (isDeployed()) {
                motor.setControl(motorControl.withOutput(0.5));
            }
        });
    }

    public Command runOut() {
        return runOnce(() -> {
            if (isDeployed()) {
                motor.setControl(motorControl.withOutput(-0.5));
            }
        });
    }

    public Command stop() {
        return runOnce(() -> {
            motor.setControl(motorControl.withOutput(0));
        });
    }

    public Command toggle() {
        return runOnce(() -> {
            motor.setControl(motorControl.withOutput(0));
            piston.toggle();
        });
    }
}