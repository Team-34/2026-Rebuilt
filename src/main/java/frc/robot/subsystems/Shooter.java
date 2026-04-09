// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Shooter extends SubsystemBase {
  private static final boolean DEBUG = false;

  enum Speed {
    STOP(0.0), HALF(0.5), FULL(0.85); // enum for flywheel speeds

    public final double value;

    Speed(final double value) {
      this.value = value;
    }
  }

  private Speed speed = Speed.STOP;

  private double testingSpeed = 0.5;
  private double testingRPS = 0.5;

  private final TalonSRX hoodMotor = new TalonSRX(43); // hood motor
  private final TalonFX masterFiringMotor = new TalonFX(42); // left
  private final TalonFX padawanFiringMotor = new TalonFX(41); // right

  private final DutyCycleOut firingMotorControl = new DutyCycleOut(0);

  private final CANcoder hoodEncoder = new CANcoder(45); // external CTRE encoder

  private final DigitalInput hoodLimitSwitch = new DigitalInput(8); // limit switch for hood (obvi)
  private final Trigger hoodAtHome = new Trigger(this::isHoodAtHome);
  private final PIDController hoodPID = new PIDController(2.5, 0.0, 0.0); // PID for hood

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
    masterConfig.CurrentLimits.SupplyCurrentLimit = 60;
    masterConfig.Voltage.withPeakForwardVoltage(Volts.of(8)).withPeakReverseVoltage(Volts.of(-8));
    masterFiringMotor.getConfigurator().apply(masterConfig);
    padawanFiringMotor.setControl(new Follower(masterFiringMotor.getDeviceID(), MotorAlignmentValue.Opposed));

    hoodMotor.setInverted(true);
    hoodMotor.configRemoteFeedbackFilter(hoodEncoder.getDeviceID(), RemoteSensorSource.CANCoder, 0, 10);
    hoodMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, 0, 10);

    hoodPID.setSetpoint(hoodEncoder.getPosition().getValueAsDouble());

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

  public Command runAtIdleCommand() {
    return runOnce(this::runAtIdle);
  }

  public Command increaseBySpeedCommand() {
    return runOnce(() -> {
      testingSpeed = MathUtil.clamp(testingSpeed + 0.025, 0.1, 1.0);
      runFiringMotor(testingSpeed);
    });
  }

  public Command decreaseBySpeedCommand() {
    return runOnce(() -> {
      testingSpeed = MathUtil.clamp(testingSpeed - 0.025, 0.1, 1.0);
      runFiringMotor(testingSpeed);
    });
  }

  public Command increaseByRPSCommand() {
    return runOnce(() -> {
      testingRPS = testingRPS + 1;
      runFiringMotorByRPS(RevolutionsPerSecond.of(testingRPS));
    });
  }

  public Command decreaseByRPSCommand() {
    return runOnce(() -> {
      testingRPS = testingRPS - 1;
      runFiringMotorByRPS(RevolutionsPerSecond.of(testingRPS));
    });
  }

  public Command setHoodMotorPercent(final double speed) {
    return runOnce(() -> moveHoodMotorPercent(speed));
  }

  public Command runFiringMotorByRPSCommand(final AngularVelocity rps) {
    return runOnce(() -> runFiringMotorByRPS(rps));
  }

  public static double distanceToFiringSpeed(final Distance distance) {
    final var x = distance.in(Inches);
    final var x_2 = x * x;
    final var x_3 = x_2 * x;

    return 0.167 + (7.31e-03 * x) + (-4.24e-05 * x_2) + (1.11e-07 * x_3);
  }

  public static AngularVelocity distanceToRPS(final Distance distance) {
    final var x = distance.in(Inches);
    final var x_2 = x * x;
    final var x_3 = x_2 * x;
    final var rps = 26.6 + (0.369 * x) + (-1.49e-03 * x_2) + (4.02e-06 * x_3);
    return RotationsPerSecond.of(rps);
  }

  public Command shootBySpeedCommand() {
    return runEnd(() -> {
      final var newDistanceToHub = vision.getDistanceToHub();
      if (newDistanceToHub.isPresent()) {
        cachedHubDistance = newDistanceToHub;
      }

      cachedHubDistance.ifPresentOrElse(distance -> {
        final var speed = distanceToFiringSpeed(distance);
        runFiringMotor(speed);

        if (DEBUG) {
          SmartDashboard.putString("Shooter: Distance to Hub", distance.toLongString());
          SmartDashboard.putNumber("Shooter: Calculated Shooter Speed", speed);
        }
      }, this::runAtIdle);
    }, () -> {
      cachedHubDistance = Optional.empty();
      runAtIdle();
    });
  }

  public Command shootByRPSCommand() {
    return runEnd(() -> {
      final var newDistanceToHub = vision.getDistanceToHub();
      if (newDistanceToHub.isPresent()) {
        cachedHubDistance = newDistanceToHub;
      }

      cachedHubDistance.ifPresentOrElse(distance -> {
        final AngularVelocity rps = distanceToRPS(distance);
        runFiringMotorByRPS(rps);

        if (DEBUG) {
          SmartDashboard.putString("Shooter: Distance to Hub ", distance.toString());
          SmartDashboard.putString("Shooter: Calculated Shooter RPS", rps.toString());
        }
      }, this::runAtIdle);
    }, () -> {
      cachedHubDistance = Optional.empty();
      runAtIdle();
    });
  }

  public Command stop() {
    return runOnce(masterFiringMotor::stopMotor);
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
    final var percentOutput = MathUtil.clamp(
      hoodPID.calculate(hoodEncoder.getPosition().getValue().in(Rotations)),
      -1.0,
      1.0);
    this.hoodMotor.set(TalonSRXControlMode.PercentOutput, percentOutput);

    if (DEBUG) {
      SmartDashboard.putNumber("Shooter: Speed %", masterFiringMotor.getDutyCycle().getValueAsDouble() * 100);
      SmartDashboard.putString("Shooter: Velocity", masterFiringMotor.getVelocity().getValue().toLongString());
      // SmartDashboard.putNumber("external encoder units",
      // hoodEncoder.getPosition().getValueAsDouble());
      // SmartDashboard.putNumber("Hood Motor pos: ",
      // hoodMotor.getSelectedSensorPosition());
      // SmartDashboard.putNumber("Hood Motor Velocity: ", hoodPID.getSetpoint());
      // SmartDashboard.putBoolean("limit switch: ", hoodLimitSwitch.get());

      SmartDashboard.putNumber("Shooter: Hood Motor Output", percentOutput);
    }
  }

  private void runFiringMotor(final double percent) {
    this.masterFiringMotor.setControl(firingMotorControl.withOutput(percent));
  }

  private void runFiringMotorByRPS(final AngularVelocity rps) {
    this.masterFiringMotor.setControl(velocityControl.withFeedForward(0.1).withVelocity(rps));
  }

  private void moveHoodMotorRotations(final double rotations) {
    this.hoodPID.setSetpoint(rotations);
  }

  private void moveHoodMotorPercent(final double speed) {
    this.hoodMotor.set(TalonSRXControlMode.PercentOutput, speed);
  }

  private void zeroHoodEncoder() {
    this.hoodEncoder.setPosition(0.0);
    this.hoodPID.setSetpoint(0.0);
  }

  private void runAtIdle() {
    this.masterFiringMotor.set(0.1);
  }
}
