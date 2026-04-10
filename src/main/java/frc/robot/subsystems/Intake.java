package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Maths;

public class Intake extends SubsystemBase {
  private static final boolean DEBUG = false;

  enum DeploymentState {
    DEPLOYED(Rotations.of(2.65)), RETRACTED(DEPLOY_MIN_ROTATIONS), BUMPER(Rotations.of(0.5)); //original val: 0.914

    public final Angle rotations;

    DeploymentState(final Angle rotations) {
      this.rotations = rotations;
    }
  }

  private final TalonFXS rollerMotor = new TalonFXS(60);
  private final DutyCycleOut rollerMotorControl = new DutyCycleOut(0);

  private static final Angle DEPLOY_MIN_ROTATIONS = Rotations.zero();
  private static final Angle DEPLOY_MAX_ROTATIONS = Rotations.of(2.7);
  private final TalonFXS deployMotor = new TalonFXS(61);
  private final PositionVoltage deployPositionControl = new PositionVoltage(0);
  private final DutyCycleOut deployMotorControl = new DutyCycleOut(0);
  private final CANcoder deployEncoder = new CANcoder(62);
  private DeploymentState deploymentState = DeploymentState.RETRACTED;

  public Intake() {
    final TalonFXSConfiguration rollerConfig = new TalonFXSConfiguration();
    rollerConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    rollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rollerMotor.getConfigurator().apply(rollerConfig);

    deployEncoder.setPosition(Rotations.zero());

    final TalonFXSConfiguration deployConfig = new TalonFXSConfiguration();
    deployConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    deployConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    //// removed because it prevented the Minion from drawing enough current.
    deployConfig.CurrentLimits
      .withStatorCurrentLimit(Amps.of(100))
      .withStatorCurrentLimitEnable(true)
      .withSupplyCurrentLimit(Amps.of(90))
      .withSupplyCurrentLimitEnable(true);
    deployConfig.ExternalFeedback.withExternalFeedbackSensorSource(ExternalFeedbackSensorSourceValue.RemoteCANcoder)
        .withFeedbackRemoteSensorID(62);
    deployConfig.Slot0.withKP(10).withKI(0.0).withKD(0.2);
    deployMotor.getConfigurator().apply(deployConfig);
  }

  public Command runIn() {
    return runOnce(() -> runRollerMotorAt(0.5));
  }

  public Command runOut() {
    return runOnce(() -> runRollerMotorAt(-0.5));
  }

  public Command stop() {
    return runOnce(rollerMotor::stopMotor);
  }

  public Command haltDeployment() {
    return runOnce(deployMotor::stopMotor);
  }

  // 2 3/4 rotations of encoder to deployment, motor to encoder gr is 22/12

  public Command cycleDeployment() {
    return runOnce(() -> {
      this.deploymentState = switch (this.deploymentState) {
        case RETRACTED -> DeploymentState.BUMPER;
        case BUMPER -> DeploymentState.DEPLOYED;
        case DEPLOYED -> DeploymentState.BUMPER;
        default -> DeploymentState.RETRACTED;
      };
      rotateDeployMotorTo(this.deploymentState.rotations);
      if (deploymentState == DeploymentState.DEPLOYED) {
        runRollerMotorAt(0.5);
      } else {
        rollerMotor.stopMotor();
      }
    });
  }

  public Command deployByPower(final double power) {
    return runOnce(() -> {
      deployMotor.setControl(deployMotorControl.withOutput(power));

      if (DEBUG) {
        SmartDashboard.putNumber("intake deploy motor input", power);
      }
    });
  }

  public void runRollerMotorAt(final double speed) {
    if (deploymentState == DeploymentState.DEPLOYED) {
      rollerMotor.setControl(rollerMotorControl.withOutput(speed));
    }
  }

  private void rotateDeployMotorTo(final Angle rotations) {
    rollerMotor.stopMotor();

    final var safeRotations = Maths.clamp(rotations, DEPLOY_MIN_ROTATIONS, DEPLOY_MAX_ROTATIONS);
    deployMotor.setControl(deployPositionControl.withPosition(safeRotations));

    if (DEBUG) {
      SmartDashboard.putString("intake deploy commanded rotations", rotations.toLongString());
      SmartDashboard.putString("intake deploy commanded SAFE rotations", safeRotations.toLongString());
    }
  }

  @Override
  public void periodic() {
    if (DEBUG) {
      SmartDashboard.putString("intake deployment state", deploymentState.toString());
      SmartDashboard.putString("intake deploy encoder position", deployEncoder.getPosition().getValue().toLongString());
      SmartDashboard.putNumber("intake deploy motor output", deployMotor.get());
      SmartDashboard.putString("intake deploy motor position", deployMotor.getPosition().getValue().toLongString());
      SmartDashboard.putString("intake deploy motor voltage", deployMotor.getMotorVoltage().getValue().toLongString());
    }
  }
}
