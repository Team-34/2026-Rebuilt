package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Intake extends SubsystemBase {
  
    private final DoubleSolenoid piston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);

    public final TalonFXS Motor = new TalonFXS(80);
    public final DutyCycleOut motorControl = new DutyCycleOut(0);

    public void robotInit() {
       
    }

    public void teleopPeriodic() {
        // periodic code can go here
    }

    public Command runIn() {
        return runOnce(() -> {
             Motor.setControl(motorControl.withOutput(0.5));
        });
    }

    public Command runOut() {
        return runOnce(() -> {
             Motor.setControl(motorControl.withOutput(-0.5));
        });
    }

    public Command stop() {
        return runOnce(() -> {
            Motor.setControl(motorControl.withOutput(0));
        });
    }

    public Command Toggle() {
        piston.toggle();
        return null;
    }
}