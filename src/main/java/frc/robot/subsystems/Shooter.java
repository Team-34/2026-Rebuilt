// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

public class Shooter extends SubsystemBase {
  enum Speed {
    Stop(0.0),
    Half(0.5),
    Full(1.0);

    public final double value;

    Speed(double value) {
      this.value = value;
    }
  }

  private Speed speed;
  private TalonFXS leftMotor = new TalonFXS(20);
  private TalonFXS rightMotor = new TalonFXS(21);
  TalonFXSConfiguration configs = new TalonFXSConfiguration();

  public Shooter() {
    this.configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    this.rightMotor.set(this.leftMotor.get());
    this.rightMotor.getConfigurator().apply(configs);
  }

  /**
   * Cyles thought 3 different speeds. Stop -> Half -> Full -> Stop -> ...
   *
   * @returns The command that switches the speed to the next in the cycle.
   */
  
  public Command cycleSpeedCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(() -> {
      switch (this.speed) {
        case Stop:
          this.speed = Speed.Half;
          break;
        case Half:
          this.speed = Speed.Full;
          break;
        case Full:
        default:
          this.speed = Speed.Stop;
          break;
      }
    });
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
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
