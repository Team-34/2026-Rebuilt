// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.BrushedMotorWiringValue;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FXSShooter extends SubsystemBase {
  enum Speed {
    STOP(0.0), HALF(0.5), FULL(1.0);

    public final double value;

    Speed(double value) {
      this.value = value;
    }
  }

  // private final TalonSRX aimer = new TalonSRX(23); ////old object of TalonSRX
  // motor for hood.
  private Speed speed = Speed.STOP;
  private final TalonFX master = new TalonFX(22);// left
  private final TalonFX padawan = new TalonFX(21);// right
  private final TalonFXS hoodMotor = new TalonFXS(24);// trajectory motor

  private final Encoder hoodEncoder = new Encoder(1, 0);// built-in encoder on PlG
  private final CANcoder externalCancoder = new CANcoder(25);// external CTRE encoder

  private final DutyCycleOut hoodMotorControl = new DutyCycleOut(0);

  private final DigitalInput limitSwitch = new DigitalInput(4);

  /**
  * 
  */
  public FXSShooter() {
    TalonFXConfiguration masterConfig = new TalonFXConfiguration();
    TalonFXSConfiguration hoodConfiguration = new TalonFXSConfiguration();
    masterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    master.getConfigurator().apply(masterConfig);
    padawan.setControl(new Follower(master.getDeviceID(), MotorAlignmentValue.Opposed));

    hoodConfiguration.ExternalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.RemoteCANcoder;
    hoodConfiguration.ExternalFeedback.FeedbackRemoteSensorID = externalCancoder.getDeviceID();
    hoodConfiguration.Slot0.withKP(10).withKI(0).withKD(0);
    hoodConfiguration.Commutation.MotorArrangement = MotorArrangementValue.Brushed_DC;
    hoodConfiguration.Commutation.BrushedMotorWiring = BrushedMotorWiringValue.Leads_A_and_B;

    hoodMotor.getConfigurator().apply(hoodConfiguration);

    hoodMotor.setControl(hoodMotorControl.withOutput(Speed.STOP.value));

    // aimer.config_kP(0, 10, 10);
    // aimer.config_kI(0, 0.0, 10);
    // aimer.config_kD(0, 0.0, 10);
    // aimer.config_kF(0, 0.0, 10);
    // aimer.configRemoteFeedbackFilter(external_cancoder.getDeviceID(),
    // RemoteSensorSource.CANCoder, 0, 10);
    // aimer.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, 0, 10);
    // aimer.configMotionCruiseVelocity(1000, 10);
    // aimer.configMotionAcceleration(500, 10);

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

  private void runFiringMotor(double motor_speed) {
    this.master.set(motor_speed);
  }

  public Command setPosition() {
    return runOnce(() -> {
      moveAimingMotorRot(2);
    });
  }

  public Command setPrecent(double run_speed) {
    return runOnce(() -> {
      moveAimingMotorPercent(run_speed);
    });
  }

  private void moveAimingMotorRot(double motor_rot) {

    double position = motor_rot;
    this.hoodMotor.setPosition(position);
    // this.aimer.set(TalonSRXControlMode.Position, position);
  }

  private void moveAimingMotorPercent(double motor_speed) {

    this.hoodMotor.setControl(hoodMotorControl.withOutput(motor_speed));
    // this.aimer.set(TalonSRXControlMode.PercentOutput, motor_speed);
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
    SmartDashboard.putNumber("built-in encoder units", hoodEncoder.get());
    SmartDashboard.putNumber("external encoder units", externalCancoder.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Hood Motor pos: ", hoodMotor.get());
    SmartDashboard.putBoolean("limit switch: ", limitSwitch.get());
    // SmartDashboard.putNumber("hood motor pos: ",
    // aimer.getSelectedSensorPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
