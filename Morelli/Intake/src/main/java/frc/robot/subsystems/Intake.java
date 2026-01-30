package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class Intake extends SubsystemBase {
  
    private final Solenoid intakeSingle = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    private final DoubleSolenoid intakeDouble = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);

    public TalonFXS m_minionMotor = new TalonFXS(10);
    public DutyCycleOut m_motorControl = new DutyCycleOut(10);

    public void robotInit() {
       
    }

    public void teleopPeriodic() {
        // periodic code can go here
    }

    public Command runIn() {
        return runOnce(() -> {
             m_minionMotor.setControl(m_motorControl.withOutput(0.5));
        });
   
    }

    public Command runOut() {
        
        return runOnce(() -> {
             m_minionMotor.setControl(m_motorControl.withOutput(-0.5));
        });
       
    }

    public Command stop() {
        return runOnce(() -> {
            m_minionMotor.setControl(m_motorControl.withOutput(0));
        });
    }

    public Command toggleIntakePneumatics() {
        intakeSingle.toggle();
        intakeDouble.toggle();
        return null;
    }
}