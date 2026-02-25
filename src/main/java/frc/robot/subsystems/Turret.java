package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;

public class Turret extends SubsystemBase {
  private final TalonFXS motor = new TalonFXS(50);
  private final PositionVoltage positionControl = new PositionVoltage(0);
  private final DigitalInput limitSwitch = new DigitalInput(3);

  public Turret() {
    final var config = new TalonFXSConfiguration();
    config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // <-- May be removed if needed
    config.Slot0.kP = 2;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;
    motor.getConfigurator().apply(config);
  }

  /**
   * @param position The position to move the turret to. As a percentage of the full rotation (0.0 to 1.0).
   * @return Moves the turret to the specified position.
   */
  public Command turretByPositionCommand(final double position) {
    return runOnce(() -> {
      motor.setControl(positionControl.withPosition(MathUtil.clamp(position, 0.0, 0.5)));
    });
  }

  /**
   * @param power The power to give to the motor.
   * @return Moves the motor.
   */
  public Command turretByPowerCommand(final double power) {
    return runEnd(() -> motor.set(power), () -> motor.stopMotor());
  }

  /**
   * @param angle The angle to swivel the turret by.
   * @return Moves the turret by the specified angle.
   */
  public Command swivelByCommand(final Angle angle) {
    return run(() -> {
      final var clamped = Degrees.of(MathUtil.clamp(angle.in(Degrees), 0, 180));
      final var adjustment = mechanismAngleToMotorAngle(clamped);
      
      final var currentMotorPosition = motor.getPosition().getValue();
      final var newMotorPosition = currentMotorPosition.plus(adjustment);
      motor.setControl(positionControl.withPosition(newMotorPosition));
    });
  }

  private boolean isAtZeroPosition() {
    return !limitSwitch.get();
  }

  private Angle mechanismAngleToMotorAngle(final Angle angle) {
    final double GEAR_RATIO = 99.0 / 18.0;

    return angle.times(GEAR_RATIO);
  }

  private void resetEncoder() {
    // Set the encoder position to zero rotations
    // The argument is the new position value
    motor.setPosition(0);
  }
  
  @Override
  public void periodic() {
    if (isAtZeroPosition()) {
      resetEncoder();
    }

    // Minion motor returns the encoder in full rotations (ex. 1 unit is 1 full rotation)
    SmartDashboard.putNumber("Encoder", motor.getPosition().getValueAsDouble());
    SmartDashboard.putBoolean("Is at zero? ", isAtZeroPosition());
  }
}
