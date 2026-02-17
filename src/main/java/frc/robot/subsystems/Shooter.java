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
    STOP(0.0), HALF(0.5), FULL(1.0);

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

  private final DigitalInput hoodLimitSwitch = new DigitalInput(4);

  private final PIDController hoodPID = new PIDController(2.5, 0.0, 0.0); // PID for hood

  private double hoodSetPoint = 0.0;

  /**
  * 
  */
  public Shooter() {
    TalonFXConfiguration masterConfig = new TalonFXConfiguration();
    masterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    master.getConfigurator().apply(masterConfig);
    padawan.setControl(new Follower(master.getDeviceID(), MotorAlignmentValue.Opposed));

    hoodMotor.setInverted(true);

    hoodMotor.configRemoteFeedbackFilter(externalCancoder.getDeviceID(), RemoteSensorSource.CANCoder, 0, 10);
    hoodMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, 0, 10);
    // hoodMotor.configMotionCruiseVelocity(1000, 10);
    // hoodMotor.configMotionAcceleration(500, 10);
    setPoint = externalCancoder.getPosition().getValueAsDouble();
    hoodPID.setSetpoint(setPoint);
  }

  // right motor should go clockwise
  /**
   * Cyles thought 3 different speeds. Stop -> Half -> Full -> Stop -> ...
   *
   * @returns The command that switches the speed to the next in the cycle.
   */

  public Command cycleSpeedCommand() { // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
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

  private void runFiringMotor(double speed) {
    this.master.set(speed);
  }

  public Command setPosition(double position) {
    return runOnce(() -> {
      moveAimingMotorRot(position);
    });
  }

  public Command setPercent(double run_speed) {
    return runOnce(() -> {
      moveAimingMotorPercent(run_speed);
    });
  }

  private void moveAimingMotorRot(double motor_rot) {

    this.setPoint = motor_rot;
    // this.hoodMotor.setPosition(position);
  private void moveAimingMotorRotations(double rotations) {
    this.setPoint = rotations;
  }

  private void moveAimingMotorPercent(double motor_speed) {

    this.hoodMotor.set(TalonSRXControlMode.PercentOutput, motor_speed);
    // this.hoodMotor.setControl(hoodMotorControl.withOutput(motor_speed));
  }

  private void ZeroEncoder() {
    externalCancoder.setPosition(0.0);
    this.setPoint = 0.0;
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Speed: ", master.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("external encoder units", externalCancoder.getPosition().getValueAsDouble());
    SmartDashboard.putBoolean("limit switch: ", limitSwitch.get());
    SmartDashboard.putNumber("Hood Motor pos: ", hoodMotor.getSelectedSensorPosition());
    // hoodMotor.getSelectedSensorPosition());
    var pose = MathUtil.clamp(hoodPID.calculate(externalCancoder.getPosition().getValueAsDouble(), setPoint), -1.0,
        1.0);
    this.hoodMotor.set(TalonSRXControlMode.PercentOutput, pose);
    SmartDashboard.putNumber("Hood Motor Velocity: ", hoodPID.getSetpoint());
    SmartDashboard.putNumber("Hood Motor Output: ", pose);

    if (isHoodAtHome()) { //<= I dunno if I like this name, but you get the idea
      zeroHoodEncoder();
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
