// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;

public class Shooter extends SubsystemBase {
  enum Speed {
    STOP(0.0), HALF(0.5), FULL(0.90);

    public final double value;

    Speed(double value) {
      this.value = value;
    }
  }

  private Speed speed = Speed.STOP;
  private final TalonFXS leftMotor = new TalonFXS(22);
  private final TalonFXS rightMotor = new TalonFXS(21);

  private final TalonFX kraken = new TalonFX(25);

  public Shooter() {
    // TalonFXSConfiguration left_config = new TalonFXSConfiguration();
    // TalonFXSConfiguration right_config = new TalonFXSConfiguration();

    TalonFXConfiguration kraken_config = new TalonFXConfiguration();

    // right_config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    // left_config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

    // left_config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // var left_config_result = this.leftMotor.getConfigurator().apply(left_config);
    // var right_config_result = this.rightMotor.getConfigurator().apply(right_config);

    kraken_config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    
    kraken.set(this.speed.STOP.value);

    // SmartDashboard.putString("left shooter motor config: ", left_config_result.getName());
    // SmartDashboard.putString("right shooter motor config: ", right_config_result.getName());
    this.leftMotor.set(this.speed.STOP.value);
    //this.rightMotor.setControl(new Follower(this.leftMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    // this.leftMotor.set(this.speed.STOP.value);
    // this.rightMotor.set(this.speed.STOP.value);

  }

  /**
   * Cyles thought 3 different speeds. Stop -> Half -> Full -> Stop -> ...
   *
   * @returns The command that switches the speed to the next in the cycle.
   */

  private void runMotor(double motor_speed) {
    this.kraken.set(motor_speed);
  }

  public Command cycleSpeedCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(() -> {
      this.speed = switch (this.speed) {
        case STOP -> Speed.HALF;
        case HALF -> Speed.FULL;
        case FULL -> Speed.STOP;
        default -> Speed.STOP;
      };
      runMotor(this.speed.value);
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
    SmartDashboard.putNumber("Shooter Speed: ", kraken.getVelocity().getValueAsDouble());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
