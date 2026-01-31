package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.TalonFXS;

public class Turret extends SubsystemBase {
    
    TalonFXS turretMotor = new TalonFXS(50);

    PIDController turretPID = new PIDController(0.5, 0.5, 0.5);

    BooleanSupplier atSetpoint = () -> turretPID.atSetpoint();

    public Turret() {
    }

    /**
     * @param setpoint The setpoint to move the turret to.
     * @return Moves the turret to the setpoint.
     */
    public Command turretBySetpointCommand(double setpoint) {
        return run
        (
           () -> {
            turretPID.setSetpoint(setpoint);
           }
        ).until(atSetpoint);
    }

    public double getSetpoint() {
        return turretPID.getSetpoint();
    }


    /**
     * @param power The power to give to the motor.
     * @return Moves the motor.
     */
    public Command turretByPowerCommand(double power) {
        return runEnd
        (
           () -> {
            turretMotor.set(power);
           },
           () -> {
            turretMotor.stopMotor();
           }
        );
    }

    /**
     * @return The periodic method of the turret. As of now, it should be constantly moving the turret to the setpoint.
     *
     */
    public void periodic() {
        turretMotor.set(
            turretPID.calculate(
                turretMotor.getPosition().getValueAsDouble()
                , turretPID.getSetpoint()
            )
        );
    }

}
