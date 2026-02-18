// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  enum Speed {
    STOP(0.0), HALF(0.5), FULL(1.0); // enum for flywheel speeds

    public final double value;

    Speed(double value) {
      this.value = value;
    }
  }

  private Speed speed = Speed.STOP;

  private final TalonSRX hoodMotor = new TalonSRX(23); // hood motor
  private final TalonFX masterFiringMotor = new TalonFX(22); // left
  private final TalonFX padawanFiringMotor = new TalonFX(21); // right

  private final CANcoder hoodEncoder = new CANcoder(25); // external CTRE encoder

  private final DigitalInput hoodLimitSwitch = new DigitalInput(4); // limit switch for hood (obvi)

  private final PIDController hoodPID = new PIDController(2.5, 0.0, 0.0); // PID for hood

  private double hoodSetPoint = 0.0; // setpoint for hood position in rotations

  public Shooter() {
    TalonFXConfiguration masterConfig = new TalonFXConfiguration();
    masterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    masterFiringMotor.getConfigurator().apply(masterConfig);
    padawanFiringMotor.setControl(new Follower(masterFiringMotor.getDeviceID(), MotorAlignmentValue.Opposed));

    hoodMotor.setInverted(true);
    hoodMotor.configRemoteFeedbackFilter(hoodEncoder.getDeviceID(), RemoteSensorSource.CANCoder, 0, 10);
    hoodMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, 0, 10);
    
    hoodSetPoint = hoodEncoder.getPosition().getValueAsDouble();

    hoodPID.setSetpoint(hoodSetPoint);
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

  
  public Command setHoodPosition(double position) {
    return runOnce(() -> {
      moveHoodMotorRotations(position);
    });
  }
  
  public Command setHoodMotorPercent(double speed) {
    return runOnce(() -> {
      moveHoodMotorPercent(speed);
    });
  }

  /**
   * Checks if the hood is at its "home" position, which is defined as the position where the hood encoder reads 0.0. 
   * This can be used to determine if the hood has reached a known reference point, such as when a limit switch is triggered or after a reset.
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean isHoodAtHome() {
    return hoodEncoder.getPosition().getValueAsDouble() == 0.0;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Speed: ", masterFiringMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("external encoder units", hoodEncoder.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Hood Motor pos: ", hoodMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Hood Motor Velocity: ", hoodPID.getSetpoint());
    SmartDashboard.putBoolean("limit switch: ", hoodLimitSwitch.get());

    var pose = MathUtil.clamp(hoodPID.calculate(hoodEncoder.getPosition().getValueAsDouble(), hoodSetPoint), -1.0, 1.0);
    this.hoodMotor.set(TalonSRXControlMode.PercentOutput, pose);
    SmartDashboard.putNumber("Hood Motor Output: ", pose);
    

    if (isHoodAtHome()) {
      zeroHoodEncoder();
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

    private void runFiringMotor(double speed) {
    this.masterFiringMotor.set(speed);
  }

  private void moveHoodMotorRotations(double rotations) {
    this.hoodSetPoint = rotations;
  }

  private void moveHoodMotorPercent(double speed) {
    this.hoodMotor.set(TalonSRXControlMode.PercentOutput, speed);
  }

  private void zeroHoodEncoder() {
    hoodEncoder.setPosition(0.0);
    this.hoodSetPoint = 0.0;
  }
}
