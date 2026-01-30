package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.TalonFXS;

import frc.robot.LimelightHelpers;

public class Turret extends SubsystemBase {
    
    TalonFXS turretMotor = new TalonFXS(50);

    PIDController turretPID = new PIDController(0.5, 0.5, 0.5);

    BooleanSupplier atSetpoint = () -> turretPID.atSetpoint();

    public Turret() {
    }

    // IDs for the apriltags on both outposts, using 1 for testing purposes
    //int[] validIDs = {9, 10, 25, 26};
    int[] validIDs = {1};

    // Get the area of the target, in a percentage of 0 to 100
    double ta = LimelightHelpers.getTA("");

    LimelightHelpers.PoseEstimate poseEstimate;

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

        LimelightHelpers.SetRobotOrientation("", 0, 0, 0, 0, 0, 0);
    }

}
