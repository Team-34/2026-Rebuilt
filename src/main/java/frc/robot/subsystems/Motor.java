// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Motor extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public Motor() {}

  SparkMax m_motor = new SparkMax(4, MotorType.kBrushless);

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  public Command MoveMotor(double speed) {
    return runEnd(
      () -> 
      {
        m_motor.set(1.0);
      }, 
      () -> 
      {
        m_motor.stopMotor();
      });
  }
// Stops the motor
  public Command StopMotor(double speed) {
    return runEnd(
      () -> 
      {
        m_motor.stopMotor();;
      }, 
      () -> 
      {
        m_motor.stopMotor();
      });
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
