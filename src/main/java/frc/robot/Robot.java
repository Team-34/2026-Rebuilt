// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final Field2d m_field = new Field2d();
  private final RobotContainer m_robotContainer;
  /* log and replay timestamp and joystick data */
  private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
      .withTimestampReplay()
      .withJoystickReplay();

  public Robot() {
    this.m_robotContainer = new RobotContainer();

  }

  @Override
  public void robotPeriodic() {
     SmartDashboard.putData("Swerve Drive", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");
        
        builder.addDoubleProperty("Front Left Angle", () -> m_robotContainer.drivetrain.getModule(1).getSteerMotor().getPosition().getValueAsDouble() % 360, null);
        builder.addDoubleProperty("Front Left Velocity", () -> m_robotContainer.drivetrain.getModule(1).getDriveMotor().getVelocity().getValueAsDouble(), null);
        
        builder.addDoubleProperty("Front Right Angle",  () -> m_robotContainer.drivetrain.getModule(2).getSteerMotor().getPosition().getValueAsDouble() % 360, null);
        builder.addDoubleProperty("Front Right Velocity",  () -> m_robotContainer.drivetrain.getModule(2).getDriveMotor().getVelocity().getValueAsDouble(), null);
        
        builder.addDoubleProperty("Back Left Angle", () -> m_robotContainer.drivetrain.getModule(3).getSteerMotor().getPosition().getValueAsDouble() % 360, null);
        builder.addDoubleProperty("Back Left Velocity", () -> m_robotContainer.drivetrain.getModule(2).getDriveMotor().getVelocity().getValueAsDouble(), null);
        
        builder.addDoubleProperty("Back Right Angle", () -> m_robotContainer.drivetrain.getModule(3).getSteerMotor().getPosition().getValueAsDouble() % 360, null);
        builder.addDoubleProperty("Back Right Velocity", () -> m_robotContainer.drivetrain.getModule(2).getDriveMotor().getVelocity().getValueAsDouble(), null);
        
        builder.addDoubleProperty("Robot Angle", () -> m_robotContainer.drivetrain.getPigeon2().getYaw().getValueAsDouble(), null);
    }
});
    SmartDashboard.putData("Field", m_field);
    this.m_timeAndJoystickReplay.update();
    m_field.setRobotPose(m_robotContainer.limelightHelpers.getBotPose2d(""));
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    this.m_autonomousCommand = this.m_robotContainer.getAutonomousCommand();

    if (this.m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(this.m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (this.m_autonomousCommand != null) {
      CommandScheduler.getInstance().cancel(this.m_autonomousCommand);
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
