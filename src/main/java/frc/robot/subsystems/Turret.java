package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.util.Maths;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static edu.wpi.first.units.Units.Degrees;

import java.util.List;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Turret extends SubsystemBase {
  private static final double GEAR_RATIO = 99.0 / 18.0; // turret : motor

  private final TalonFXS motor = new TalonFXS(50);
  private final PositionVoltage positionControl = new PositionVoltage(0);
  private final DigitalInput limitSwitch = new DigitalInput(9);
  
    private final Game game;
  
    /**
     * Creates a new {@code Turret} instance.
     */
    public Turret(final Game game) {
      final var config = new TalonFXSConfiguration();
      config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
      config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      config.Slot0.kP = 3.5;
      config.Slot0.kI = 0;
      config.Slot0.kD = 0.1;
      config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
      motor.getConfigurator().apply(config);
      
      this.game = game;
    }
  
    public Command pointAtHubCommand() {
      return runEnd(() -> {
      List<Integer> tags = game.getHubTagIDs();
      RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("");
      for (LimelightHelpers.RawFiducial fiducial : fiducials) {
        if (tags.contains(fiducial.id)) {
          var turretPosition = motorAngleToMechanismAngle(motor.getPosition().getValue());
          var tx = Degrees.of(fiducial.txnc).unaryMinus();
          var newTurretPosition = Maths.clamp(tx.plus(turretPosition), Degrees.zero(), Degrees.of(180.0));
          var motorPosition = mechanismAngleToMotorAngle(newTurretPosition);
          motor.setControl(positionControl.withPosition(motorPosition));
          break;
        }
      }
    }, () -> {
      motor.stopMotor();
    });
  }

  /**
   * Converts turret position (in rotations) to the equivalent motor position.
   * 
   * @param rotations Turret position (in rotations)
   * @return Motor equivalent of {@code rotations}
   */
  public static double mechanismPositionToMotorPosition(final double rotations) {
    return rotations * GEAR_RATIO;
  }

  /**
   * Converts motor position (in rotations) to the equivalent turret position.
   * 
   * @param rotations Motor position (in rotations)
   * @return Turret equivalent of {@code rotations}
   */
  public static double motorPositiontoMechanismPosition(final double rotations) {
    return GEAR_RATIO / rotations;
  }

  /**
   * Converts turret position angle to the equivalent motor position angle.
   * 
   * @param rotations Turret position angle
   * @return Motor equivalent of {@code angle}
   */
  public static Angle mechanismAngleToMotorAngle(final Angle angle) {
    return angle.times(GEAR_RATIO);
  }

  /**
   * Converts motor position angle to the equivalent turret position angle.
   * 
   * @param rotations Motor position angle
   * @return Turret equivalent of {@code angle}
   */
  public static Angle motorAngleToMechanismAngle(final Angle angle) {
    return angle.div(GEAR_RATIO);
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
  public Command swivelToPositionCommand(final Angle degree) {
    return runOnce(() -> {
      final Angle encoder = motor.getPosition().getValue();
      // final var clamped = MathUtil.clamp(rotations, 0.0, 0.5);
      // motor.setControl(positionControl.withPosition(mechanismPositionToMotorPosition(clamped)));

      motor.setControl(positionControl.withPosition(mechanismAngleToMotorAngle(degree)));
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
   * **UPDATED** In theory, if the given encoder value is below the arbitrary
   * limit and we are moving at a velocity that would make us move past the limit
   * 
   * @param power The power to move the motor with, between -1 and 1. Positive
   *              power is counterclockwise; negative is clockwise.
   * @return Command to swivel the turret. POSITIVE CCW NEG CW
   */
  public Command swivelByPowerCommand(final double power) {
    return runEnd(() -> motor.set(power), motor::stopMotor).until(() -> {
      final var turretPos = motorAngleToMechanismAngle(motor.getPosition().getValue());
      final double motorVoltage = motor.getMotorVoltage().getValueAsDouble();
      final boolean highCheck = turretPos.gte(Degrees.of(90));
      final boolean lowCheck = turretPos.lte(Degrees.of(0));

      SmartDashboard.putNumber("Motor Voltage Command", motorVoltage);
      SmartDashboard.putNumber("Turret Position Command", turretPos.in(Degrees));
      SmartDashboard.putBoolean("High Check?", highCheck);
      SmartDashboard.putBoolean("Low Check?", lowCheck);

      return (highCheck && motorVoltage > 0) || (lowCheck && motorVoltage < 0);
    });
  }

  public Command stopMotor() {
    return runOnce(() -> {
      motor.stopMotor();
    });
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
   * The high zeroing method. Will only be ran if the turret ring's position is
   * near (but below) 360 and greater than 225.
   * <p>
   * The (supposed) logic is: move positive until we turn over 360 and reach zero.
   *
   * @param speed The speed to move the motor at for zeroing. Keep this value VERY
   *              low, but not low enough to stall the motor.
   */
  private void findZeroHigh(final double speed) {
    final double bigDeg = ((mechanismAngleToMotorAngle(motor.getPosition().getValue()).div(GEAR_RATIO)).abs(Degrees));
    final int LIMIT = 225;
    if (isAtZeroPosition() == false && bigDeg >= LIMIT) {
      motor.set(speed);
    }
  }

  /**
   * The low zeroing method. Will only be ran if the turret ring's position is
   * near (but above) 0 but above 90.
   * <p>
   * The (supposed) logic is: move negative until we reach zero.
   * 
   * @param speed The speed to move the motor at for zeroing. Keep this value VERY
   *              low, but not low enough to stall the motor.
   */
  private void findZeroLow(final double speed) {
    final double bigDeg = ((mechanismAngleToMotorAngle(motor.getPosition().getValue()).div(GEAR_RATIO)).abs(Degrees));
    final int LIMIT = 135;
    if (!isAtZeroPosition() && bigDeg <= LIMIT) {
      motor.set(-speed);
    }
  }

  /**
   * The main command for zeroing the turret. This is meant to be employed on init
   * of either teleop or the robot.
   * <p>
   * The logic behind this method:
   * <p>
   * If we are below 360 and within the given {@code highTolerance}, move positive
   * (or CW/CCW, will need testing) until we turn over 360 and reach zero.
   * <p>
   * If we are above 0 and within the given {@code lowTolerance}, move negative
   * (or CW/CCW, will need testing) until we reach zero.
   * <p>
   * 
   * @param lowTolerance  The lower tolerance for zeroing. Keep this value below
   *                      120 but above a feasible number for the turret to
   *                      actually be at.
   * @param highTolerance The higher tolerance for zeroing. Keep this value below
   *                      120 but above a feasible number for the turret to
   *                      actually be at.
   * @return Command to move towards zero, and stop once it is reached.
   */
  public Command findZeroCommand(final double speed) {
    return runEnd(() -> motor.set(-speed), () -> motor.stopMotor())
              .until(() -> isAtZeroPosition());
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
   * Sets the encoder position to zero rotations. This does not move the motor; it
   * tells the encoder that it should consider its current position to be zero
   * (0).
   */
  private void resetEncoder() {
    motor.setPosition(0);
  }

  @Override
  public void periodic() {
    if (isAtZeroPosition()) {
      resetEncoder();
    }

    // pointAtFiducial(1);

    // Minion motor returns the encoder in full rotations (ex. 1 unit is 1 full
    // rotation)
    SmartDashboard.putNumber("Turret Motor Voltage", motor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Turret Vel", motor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Encoder", motor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Turret Mechanism Pos",
        motorAngleToMechanismAngle(motor.getPosition().getValue()).in(Degrees));
    SmartDashboard.putBoolean("Is at zero? ", isAtZeroPosition());
  }
}