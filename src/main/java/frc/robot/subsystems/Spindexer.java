package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spindexer extends SubsystemBase {
  private final TalonFX motor = new TalonFX(55);
  private final DutyCycleOut control = new DutyCycleOut(0);

  public Command spin() {
    return runOnce(() -> {
      motor.setControl(this.control.withOutput(0.5));
    });
  }

  public Command stop() {
    return runOnce(() -> {
      motor.setControl(this.control.withOutput(0));
    });
  }

  public Command spinReverse() {
    return runOnce(() -> {
      motor.setControl(this.control.withOutput(-0.5));
    });
  }
}
