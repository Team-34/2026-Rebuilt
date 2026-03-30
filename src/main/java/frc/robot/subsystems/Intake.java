package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFXS;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.units.measure.Distance;

public class Intake extends SubsystemBase {

    enum DeploymentState {
      DEPLOYED(Inches.of(10.7)), RETRACTED(Inches.of(0)), BUMPER(Inches.of(4));
      
      public final Distance value;
      
      DeploymentState(final Distance value) {
        this.value = value;
      }
    }

  public final TalonFXS motor = new TalonFXS(60);
  public final DutyCycleOut motorControl = new DutyCycleOut(0);
  public final TalonFXS motor2 = new TalonFXS(61);
  public final PositionVoltage positionControl = new PositionVoltage(0);
  public final double GEAR_RATIO = 2.0 / 5.0;

  private DeploymentState deploymentState = DeploymentState.RETRACTED;

  private double MotorUnitToBigGearUnits(double encoderUnits) {
    return encoderUnits * GEAR_RATIO;
  }

  public Intake() {
    final TalonFXSConfiguration config = new TalonFXSConfiguration();
    config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motor.getConfigurator().apply(config);
  }

  public Command runIn() {
    return runOnce(() -> activate(0.5));
  }

  public Command runOut() {
    return runOnce(() -> activate(-0.5));
  }

  public Command stop() {
    return runOnce(() -> {
      motor.setControl(motorControl.withOutput(0));
    });
  }

  public Command moveToInches(double inches) {
    return runOnce(() -> {
      //
    });
  }

    public Command toggle() {
    return runOnce(() -> {
      this.deploymentState = switch (this.deploymentState) {
        case RETRACTED -> DeploymentState.BUMPER;
        case BUMPER -> DeploymentState.DEPLOYED;
        case DEPLOYED -> DeploymentState.RETRACTED;
        default -> DeploymentState.RETRACTED;
      };
    });
  }

  public void activate(final double speed) {
    motor.setControl(motorControl.withOutput(speed));
  }
}