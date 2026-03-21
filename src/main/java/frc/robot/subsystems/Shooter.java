// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Shooter extends SubsystemBase {
  enum Speed {
    STOP(0.0), HALF(0.5), FULL(0.75); // enum for flywheel speeds

    public final double value;

    Speed(final double value) {
      this.value = value;
    }
  }

  private Speed speed = Speed.STOP;

  private double testingSpeed = 0.5;

  private final TalonSRX hoodMotor = new TalonSRX(43); // hood motor
  private final TalonFX masterFiringMotor = new TalonFX(42); // left
  private final TalonFX padawanFiringMotor = new TalonFX(41); // right

  private final DutyCycleOut firingMotorControl = new DutyCycleOut(0);

  private final CANcoder hoodEncoder = new CANcoder(45); // external CTRE encoder

  private final DigitalInput hoodLimitSwitch = new DigitalInput(8); // limit switch for hood (obvi)
  private final Trigger hoodAtHome = new Trigger(this::isHoodAtHome);
  private final PIDController hoodPID = new PIDController(2.5, 0.0, 0.0); // PID for hood

  private double hoodSetPoint = 0.0; // setpoint for hood position in rotations

  private final Vision vision;

  private final VelocityVoltage velocityControl = new VelocityVoltage(0).withSlot(0);

  private Optional<Distance> cachedHubDistance = Optional.empty();

  public Shooter(final Vision vision) {
    final TalonFXConfiguration masterConfig = new TalonFXConfiguration();
    masterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    masterConfig.Slot0.kV = 0.12;
    masterConfig.Slot0.kP = 0.11;
    masterConfig.Slot0.kI = 0.0;
    masterConfig.Slot0.kD = 0.0;
    masterConfig.Slot0.kS = 0.1;
    masterConfig.Voltage.withPeakForwardVoltage(Volts.of(8)).withPeakReverseVoltage(Volts.of(-8));
    masterFiringMotor.getConfigurator().apply(masterConfig);
    padawanFiringMotor.setControl(new Follower(masterFiringMotor.getDeviceID(), MotorAlignmentValue.Opposed));

    hoodMotor.setInverted(true);
    hoodMotor.configRemoteFeedbackFilter(hoodEncoder.getDeviceID(), RemoteSensorSource.CANCoder, 0, 10);
    hoodMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, 0, 10);

    hoodSetPoint = hoodEncoder.getPosition().getValueAsDouble();

    hoodPID.setSetpoint(hoodSetPoint);

    this.vision = vision;

    hoodAtHome.onTrue(runOnce(this::zeroHoodEncoder));
  }

  /**
   * Cyles thought 3 different speeds. Stop -> Half -> Full -> Stop -> ...
   *
   * @returns The command that switches the speed to the next in the cycle.
   */

  public Command cycleSpeedCommand() {
    return runOnce(() -> {
      this.speed = switch (this.speed) {
        case STOP -> Speed.HALF;
        case HALF -> Speed.FULL;
        case FULL -> Speed.STOP;
        default -> Speed.STOP;
      };
      runFiringMotor(this.speed.value);
    });
  }

  public Command shooterByPercentCommand(final double speed) {
    return runEnd(() -> {
      runFiringMotor(speed);
    }, () -> {
      masterFiringMotor.stopMotor();
    });
  }

  public Command setHoodPosition(final double position) {
    return runOnce(() -> {
      moveHoodMotorRotations(position);
    });
  }

  public Command increaseCommand() {
    return runOnce(() -> {

      testingSpeed = MathUtil.clamp(testingSpeed + 0.025, 0.1, 1.0);
      runFiringMotor(testingSpeed);
    });
  }

  public Command decreaseCommand() {
    return runOnce(() -> {
      testingSpeed = MathUtil.clamp(testingSpeed - 0.025, 0.1, 1.0);
      runFiringMotor(testingSpeed);
    });
  }

  public Command setHoodMotorPercent(final double speed) {
    return runOnce(() -> {
      moveHoodMotorPercent(speed);
    });
  }

  public Command runFiringMotorByRPSCommand(final double rps) {
    return runOnce(() -> {
      runFiringMotorByRPS(rps);
    });
  }

  public static double distanceToFiringSpeed(final Distance distance) {
    final var x = distance.in(Inches);
    final var x_2 = x * x;
    final var x_3 = x_2 * x;

    return 0.167 + (7.31e-03 * x) + (-4.24e-05 * x_2) + (1.11e-07 * x_3);
  }

  public Command aimAndShootCommand() {
    return runEnd(() -> {
      final var newDistanceToHub = vision.getDistanceToHub();
      if (newDistanceToHub.isPresent()) {
        cachedHubDistance = newDistanceToHub;
      }

      cachedHubDistance.ifPresentOrElse(distance -> {
        SmartDashboard.putString("Distance to Hub", distance.toString());
        final var speed = distanceToFiringSpeed(distance);
        SmartDashboard.putNumber("Calculated Shooter Speed", speed);
        runFiringMotor(speed);
      }, this::runAtIdle);
    }, () -> {
      cachedHubDistance = Optional.empty();
      runAtIdle();
    });
  }

  public Command stop() {
    return runOnce(() -> {
      masterFiringMotor.stopMotor();
    });
  }

  /**
   * Checks whether the limit switch is pressed, indicating that the hood as at
   * the lowest point it can safely go.
   * 
   * @return if the hood is in its home position.
   */
  public boolean isHoodAtHome() {
    return !hoodLimitSwitch.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Speed %", masterFiringMotor.getDutyCycle().getValueAsDouble() * 100);
    SmartDashboard.putString("Shooter Velocity", masterFiringMotor.getVelocity().getValue().toString());
    // SmartDashboard.putNumber("external encoder units", hoodEncoder.getPosition().getValueAsDouble());
    // SmartDashboard.putNumber("Hood Motor pos: ", hoodMotor.getSelectedSensorPosition());
    // SmartDashboard.putNumber("Hood Motor Velocity: ", hoodPID.getSetpoint());
    // SmartDashboard.putBoolean("limit switch: ", hoodLimitSwitch.get());

    // var pos = MathUtil.clamp(hoodPID.calculate(hoodEncoder.getPosition().getValueAsDouble(), hoodSetPoint), -1.0, 1.0);
    // this.hoodMotor.set(TalonSRXControlMode.PercentOutput, pos);
    //SmartDashboard.putNumber("Hood Motor Output: ", pos);

    // if (isHoodAtHome()) {
    //   zeroHoodEncoder();
    // }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  private void runFiringMotor(final double percent) {
    this.masterFiringMotor.setControl(firingMotorControl.withOutput(percent));
  }

  private void runFiringMotorByRPS(final double rps) {
    this.masterFiringMotor.setControl(velocityControl.withFeedForward(0.1).withVelocity(RotationsPerSecond.of(rps)));
  }

  private void moveHoodMotorRotations(final double rotations) {
    this.hoodSetPoint = rotations;
  }

  private void moveHoodMotorPercent(final double speed) {
    this.hoodMotor.set(TalonSRXControlMode.PercentOutput, speed);
  }

  private void zeroHoodEncoder() {
    hoodEncoder.setPosition(0.0);
    this.hoodSetPoint = 0.0;
  }

  private void runAtIdle() {
    this.masterFiringMotor.set(0.1);
  }
}
