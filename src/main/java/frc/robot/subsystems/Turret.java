package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.BooleanSupplier;
import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;

public class Turret extends SubsystemBase {
  private static final double GEAR_RATIO = 18.0 / 99.0;

  private final TalonFXS motor = new TalonFXS(50);
  private final PositionVoltage positionControl = new PositionVoltage(0);
  private final DigitalInput limitSwitch = new DigitalInput(9);

  public Turret() {
    final var config = new TalonFXSConfiguration();
    config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // <-- May be removed if needed
    config.Slot0.kP = 1.5;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;
    motor.getConfigurator().apply(config);
  }

  /**
   * Converts turret position (in rotations) to the eqivalent motor position.
   * 
   * @param rotations Turret position (in rotations)
   * @return Motor equivalent of {@code rotations}
   */
  public static double mechanismPositionToMotorPosition(final double rotations) {
    return rotations * GEAR_RATIO;
  }

  /**
   * Converts turret position angle to the eqivalent motor position angle.
   * 
   * @param rotations Turret position angle
   * @return Motor equivalent of {@code angle}
   */
  public static Angle mechanismAngleToMotorAngle(final Angle angle) {
    return angle.times(GEAR_RATIO);
  }

  /**
   * Rotates the turret to the given absolute position.
   * <p>
   * The position is in rotations, where 0.5 is 180 degrees. The turret can only
   * rotate between 0 and 180 degrees, so the position is clamped to that range (0
   * to 0.5 rotations).
   * </p>
   * <ul>
   * <li>0° - the home position, facing the climber.</li>
   * <li>180° - facing the intake.</li>
   * </ul>
   *
   * @param rotations The position to move the turret to, in rotations (0.5 is 180
   *                  degrees), counterclockwise from the home position.
   * @return Command to swivel the turret.
   */
  public Command swivelToPositionCommand(final double rotations) {
    return runOnce(() -> {
      final var clamped = MathUtil.clamp(rotations, 0.0, 0.5);
      motor.setControl(positionControl.withPosition(mechanismPositionToMotorPosition(clamped)));
    });
  }

  /**
   * Swivels the turret by running the motor at a given power level.
   * <p>
   * The turret can only rotate between 0 and 180 degrees, so the power is applied
   * until the limit switch is triggered at the zero position or the encoder
   * reaches the maximum position for 180 degrees. Note: The power is applied
   * until the limit switch is triggered at the zero position or the encoder
   * reaches the maximum position for 180 degrees. The turret can only rotate
   * between 0 and 180 degrees, so the power is applied until one of those
   * conditions is met.
   * </p>
   * 
   * @param power The power to move the motor with, between -1 and 1. Positive
   *              power is counterclockwise; negative is clockwise.
   * @return Command to swivel the turret.
   */
  public Command swivelByPowerCommand(final double power) {
    return runEnd(() -> motor.set(power), motor::stopMotor);
  }

  /**
   * Swivels the turret by the given angle.
   * <p>
   * {@code angle} is a relative angle, not absolute. That is, if the turret is
   * currently at 45°, and {@code angle} is 3°, this method should move the turret
   * to 48°. Additionally, positive angles are counterclockwise; negative are
   * clockwise.
   * </p>
   * 
   * @param angle The angle to swivel the turret by. Positive angles are
   *              counterclockwise; negative are clockwise.
   * @return Command to swivel the turret.
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

  /**
   * The high zeroing method. Will only be ran if the turret ring's position is near (but below) 360 and greater than 225.
   * <p>
   * The (supposed) logic is: move positive until we turn over 360 and reach zero. 
   *
   * @param speed The speed to move the motor at for zeroing. Keep this value VERY low, but not low enough to stall the motor.
   */
  public Command findZeroHighCommand(double speed) {
    final double bigDeg = Math.abs((motor.getPosition().getValueAsDouble() * GEAR_RATIO) % 360);
    return runOnce(() -> {
      if (isAtZeroPosition() == false && bigDeg >= 225) {motor.set(speed);}
    });
  }

  /**
   * The low zeroing method. Will only be ran if the turret ring's position is near (but above) 0 but above 90.
   * <p>
   * The (supposed) logic is: move negative until we reach zero. 
   * 
   * @param speed The speed to move the motor at for zeroing. Keep this value VERY low, but not low enough to stall the motor.
   */
  public Command findZeroLowCommand(double speed) {
    final double bigDeg = Math.abs((motor.getPosition().getValueAsDouble() * GEAR_RATIO) % 360);
    return runOnce(() -> {
      if (isAtZeroPosition() == false && bigDeg <= 90) {motor.set(speed);}
    });
  }
  /**
   * The main command for zeroing the turret. This is meant to be employed on init of either teleop or the robot.
   * <p>
   * The logic behind this method:
   * <p>
   * If we are below 360 and within the given {@code highTolerance}, move positive (or CW/CCW, will need testing) until we turn over 360 and reach zero.
   * <p>
   * If we are above 0 and within the given {@code lowTolerance}, move negative (or CW/CCW, will need testing) until we reach zero.
   * <p>
   * @param lowTolerance The lower tolerance for zeroing. 
   * Keep this value below 120 but above a feasible number for the turret to actually be at.
   * @param highTolerance The higher tolerance for zeroing. 
   * Keep this value below 120 but above a feasible number for the turret to actually be at.
   */
  public Command findZeroCommand(int lowTolerance, int highTolerance, double speed) {
    final double bigDeg = Math.abs((motor.getPosition().getValueAsDouble() * GEAR_RATIO) % 360);
    final boolean nearHigh = MathUtil.isNear(360, bigDeg, highTolerance);
    final boolean nearLow = MathUtil.isNear(0, bigDeg, lowTolerance);
    final boolean zeroSwitch = !limitSwitch.get();

    return run(() -> {
      if (nearHigh) {
        findZeroHighCommand(speed);
      } else if (nearLow) {
        findZeroLowCommand(speed);
      }
      if (zeroSwitch) {
        motor.stopMotor();
      }
    });
  }

  /**
   * Is the turret tripping the limit switch, indicating that it is at the zero
   * position facing the climber?
   * 
   * @return {@code true} if the turret is tripping the limit switch;
   *         {@code false} otherwise.
   */
  private boolean isAtZeroPosition() {
    return !limitSwitch.get();
  }

  /**
   * Sets the encoder position to zero rotations. This does not move the motor;
   * it tells the encoder that it should consider its current position to be
   * zero (0).
   */
  private void resetEncoder() {
    motor.setPosition(0);
  }

  @Override
  public void periodic() {
    if (isAtZeroPosition()) {
      resetEncoder();
    }

    // Minion motor returns the encoder in full rotations (ex. 1 unit is 1 full
    // rotation)
    SmartDashboard.putNumber("Encoder", motor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Big Wheel Degrees", Math.abs((motor.getPosition().getValueAsDouble() * GEAR_RATIO) % 360));
    SmartDashboard.putBoolean("Is at zero? ", isAtZeroPosition());
  }
}
