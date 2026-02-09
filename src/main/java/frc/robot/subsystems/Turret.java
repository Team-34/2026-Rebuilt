package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;

public class Turret extends SubsystemBase {
    
    private final TalonFXS motor = new TalonFXS(50);

    private final PositionVoltage positionControl = new PositionVoltage(0);

    public Turret() {
    }

    /**
     * @param setpoint The setpoint to move the turret to.
     * @return Moves the turret to the setpoint.
     */
    public Command turretByPositionCommand(double position) {
        return runOnce(() -> {
          motor.setControl(positionControl.withPosition(position));
        });
    }

    /**
     * @param power The power to give to the motor.
     * @return Moves the motor.
     */
    public Command turretByPowerCommand(double power) {
        return runEnd
        (
           () -> {
            motor.set(power);
           },
           () -> {
            motor.stopMotor();
           }
        );
    }

    /**
     * @return The periodic method of the turret. As of now, it should be constantly moving the turret to the setpoint.
     *
     */
    public void periodic() {

    }

}
