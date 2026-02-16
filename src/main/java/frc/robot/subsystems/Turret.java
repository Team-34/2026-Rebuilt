package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfigAccessor;

public class Turret extends SubsystemBase {
  private final TalonFXS motor = new TalonFXS(50);
  private final PositionVoltage positionControl = new PositionVoltage(0);
  private final DigitalInput limitSwitch = new DigitalInput(4);

  public Turret() {
    final var config = new TalonFXSConfiguration();
    config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // <-- May be removed if needed
    config.Slot0.kP = 2;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;
    motor.getConfigurator().apply(config);
  }

  private void resetEncoder() {
    // Set the encoder position to zero rotations
    // The argument is the new position value
    motor.setPosition(0);
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

  private boolean isAtHome() {
    return !limitSwitch.get();
  }

  @Override
  public void periodic() {
    if (isAtHome()) {
      resetEncoder();
    }
    // Minion motor returns the encoder in full rotations (ex. 1 unit is 1 full rotation)
    SmartDashboard.putNumber("Encoder", motor.getPosition().getValueAsDouble());
    SmartDashboard.putBoolean("Is at Home? ", isAtHome());
  }
}
