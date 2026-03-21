package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spindexer extends SubsystemBase {
  private final TalonFX motor = new TalonFX(55);
  private final DutyCycleOut control = new DutyCycleOut(0);

  public Spindexer() {
    final var motorConfig = new TalonFXConfiguration();
    motorConfig.CurrentLimits.SupplyCurrentLimit = 80;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motor.getConfigurator().apply(motorConfig);
  }
  public Command spin() {
    return runOnce(() -> {
      motor.setControl(this.control.withOutput(-0.3));
    });
  }

  public Command stop() {
    return runOnce(() -> {
      motor.setControl(this.control.withOutput(0));
    });
  }

  public Command spinReverse() {
    return runOnce(() -> {
      motor.setControl(this.control.withOutput(0.3));
    });
  }

  public Command cycle() {
    return runOnce(() -> {
      motor.setControl(this.control.withOutput(0.5));
    });
  }
}
