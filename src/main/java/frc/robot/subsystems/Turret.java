package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;

public class Turret extends SubsystemBase {
  private final TalonFXS motor = new TalonFXS(50);
  private final PositionVoltage positionControl = new PositionVoltage(0);

  public Turret() {
    final var config = new TalonFXSConfiguration();
    config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // <-- May be removed if needed

    motor.getConfigurator().apply(config);
  }

  /**
   * @param setpoint The setpoint to move the turret to.
   * @return Moves the turret to the setpoint.
   */
  public Command turretByPositionCommand(final double position) {
    return runOnce(() -> {
      motor.setControl(positionControl.withPosition(position));
    });
  }

  /**
   * @param power The power to give to the motor.
   * @return Moves the motor.
   */
  public Command turretByPowerCommand(final double power) {
    return runEnd(() -> motor.set(power), () -> motor.stopMotor());
  }
}
