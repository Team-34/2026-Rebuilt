// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  // Define speeds
  private static final double SPEED_0 = 0.0;
  private static final double SPEED_1 = 0.5;
  private static final double SPEED_2 = 1.0;

  // Simulated sensor input
  private double sensorInput = 105.0; // Example sensor reading
  // Current speed index (cycle through 0, 1, 2)
  private static int speedIndex = 0;

  /** Creates a new ExampleSubsystem. */
  public Shooter() {
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command cycleSpeedCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          // Get current speed based on index
          double currentSpeed = getCurrentSpeed();
          speedIndex = (speedIndex + 1) % 3; // Cycle through 0, 1, 2
          //// running the motor code goes here...
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

  private static double getCurrentSpeed() {
    switch (speedIndex) {
      case 0:
        return SPEED_0;
      case 1:
        return SPEED_1;
      case 2:
        return SPEED_2;
      default:
        return SPEED_0;
    }
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
