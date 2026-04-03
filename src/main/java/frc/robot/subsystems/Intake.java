package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import java.util.Optional;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Power;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Intake extends SubsystemBase {

  enum DeploymentState {
    DEPLOYED(Inches.of(10.7)), RETRACTED(Inches.zero()), BUMPER(Inches.of(4));

    public final Distance value;

    DeploymentState(final Distance value) {
      this.value = value;
    }
  }

  public final TalonFXS rollerMotor = new TalonFXS(60);
  public final DutyCycleOut rollerMotorControl = new DutyCycleOut(0);

  public final TalonFXS deployMotor = new TalonFXS(61);
  public final PositionVoltage deployPositionControl = new PositionVoltage(0);
  public final DutyCycleOut deployMotorControl = new DutyCycleOut(0);
  public final double DEPLOY_GEAR_RATIO = 22.0 / 12.0;
  public final double DEPLOY_GEAR_CIRCUMFERENCE = (3.5) * Math.PI;
  public final Distance DEPLOY_MAX_EXTENSION = Inches.of(10.7);
  private final PIDController intakePID = new PIDController(1, 0.0, 0.0);
  private double intakeSetPoint = 0.0;
  private final CANcoder intakeEncoder = new CANcoder(63);

  private DeploymentState deploymentState = DeploymentState.RETRACTED;

  private double MotorUnitToBigGearUnits(final double encoderUnits) {
    return encoderUnits * DEPLOY_GEAR_RATIO;
  }

  public Intake() {
    final TalonFXSConfiguration rollerConfig = new TalonFXSConfiguration();
    rollerConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    rollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rollerMotor.getConfigurator().apply(rollerConfig);
    
    final TalonFXSConfiguration deployConfig = new TalonFXSConfiguration();
    deployConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    deployConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    deployMotor.getConfigurator().apply(deployConfig);

    //deployMotor.configRemoteFeedbackFilter(intakeEncoder.getDeviceID(), RemoteSensorSource.CANCoder, 0, 10);

    intakeSetPoint = intakeEncoder.getPosition().getValueAsDouble();

    intakePID.setSetpoint(intakeSetPoint);
  }

  public Command runIn() {
    return runOnce(() -> activate(0.5));
  }

  public Command runOut() {
    return runOnce(() -> activate(-0.5));
  }

  public Command stop() {
    return runOnce(() -> {
      rollerMotor.setControl(rollerMotorControl.withOutput(0));
    });
  }

  public Command moveToInchesCommand(final Distance inches) {
    return runOnce(() -> {
      moveToInches(inches);
    });
  }

  // 2 3/4 rotations of encoder to deployment, motor to encoder gr is 22/12

  public Command cycleDeploymentCommand() {
    return runOnce(() -> {
      this.deploymentState = switch (this.deploymentState) {
        case RETRACTED -> DeploymentState.BUMPER;
        case BUMPER -> DeploymentState.DEPLOYED;
        case DEPLOYED -> DeploymentState.RETRACTED;
        default -> DeploymentState.RETRACTED;
      };
      moveToInches(this.deploymentState.value);
    });
  }

  public Command runByPower(final double power) {
    return runOnce(() -> {
    deployMotor.setControl(deployMotorControl.withOutput(0.15));
    SmartDashboard.putNumber("intake deploy motor input", power);
    });
  }

  public void activate(final double speed) {
    rollerMotor.setControl(rollerMotorControl.withOutput(speed));
  }

  public void deploy(final double power) {
    //deployMotor.setControl(deployMotorControl.withOutput(1));
  }

  private void moveToInches(final Distance inches) {
    //deployMotor.setControl(deployPositionControl.withPosition(inches.in(Inches)));
  }

  @Override
  public void periodic() {
    //var pos =
    //MathUtil.clamp(intakePID.calculate(intakeEncoder.getPosition().getValueAsDouble(),
    //intakeSetPoint), -1.0, 1.0);
    //this.deployMotor.set(TalonFXSConfiguration.PercentOutput, pos);
    SmartDashboard.putString("intake deploy encoder position", intakeEncoder.getPosition().getValue().toLongString());
    SmartDashboard.putNumber("intake deploy motor output", deployMotor.get());
    SmartDashboard.putString("intake deploy motor voltage", deployMotor.getMotorVoltage().getValue().toLongString());
  }
}