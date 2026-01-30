// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Motor;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
  private final Shooter shooter = new Shooter();
  Motor m_motor = new Motor();

  SlewRateLimiter ForwardFilter = new SlewRateLimiter(1.7);
  SlewRateLimiter TurnFilter = new SlewRateLimiter(1.7);
  SlewRateLimiter RotateFilter = new SlewRateLimiter(1.7);

  private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired
                                                                                      // top
                                                                                      // speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                    // second
                                                                                    // max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(this.MaxSpeed * 0.1).withRotationalDeadband(this.MaxAngularRate * 0.05) // Add a 10%
                                                                                  // deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                               // motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final Telemetry logger = new Telemetry(this.MaxSpeed);

  private final CommandXboxController joystick = new CommandXboxController(0);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  /* Path follower */
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {

    NamedCommands.registerCommand("Move Motor", this.m_motor.MoveMotor(0.5));
    NamedCommands.registerCommand("Stop Motor", this.m_motor.MoveMotor(0.0));

    this.autoChooser = AutoBuilder.buildAutoChooser("Tests");
    SmartDashboard.putData("Auto Mode", this.autoChooser);

    this.configureBindings();

    // Warmup PathPlanner to avoid Java pauses
    FollowPathCommand.warmupCommand().schedule();
  }

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    this.drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        this.drivetrain.applyRequest(
            () -> this.drive.withVelocityX(this.ForwardFilter
                .calculate(-this.joystick.getLeftY() * this.MaxSpeed)) // Drive
                                                             // forward
                                                             // with
                                                             // negative
                                                             // Y
                                                             // (forward)
                .withVelocityY(this.TurnFilter.calculate(
                    -this.joystick.getLeftX() * this.MaxSpeed)) // Drive
                                                      // left
                                                      // with
                                                      // negative
                                                      // X
                                                      // (left)
                .withRotationalRate(this.RotateFilter.calculate(
                    -this.joystick.getRightX() * this.MaxAngularRate)) // Drive
                                                             // counterclockwise
                                                             // with
                                                             // negative
                                                             // X
                                                             // (left)
        ));

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
        this.drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    this.joystick.a().whileTrue(this.drivetrain.applyRequest(() -> this.brake));
    this.joystick.b().whileTrue(this.drivetrain.applyRequest(
        () -> this.point.withModuleDirection(
            new Rotation2d(-this.joystick.getLeftY(), -this.joystick.getLeftX()))));

    this.joystick.povUp().whileTrue(
        this.drivetrain.applyRequest(() -> this.forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    this.joystick.povDown()
        .whileTrue(this.drivetrain.applyRequest(
            () -> this.forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    this.joystick.back().and(this.joystick.y()).whileTrue(this.drivetrain.sysIdDynamic(Direction.kForward));
    this.joystick.back().and(this.joystick.x()).whileTrue(this.drivetrain.sysIdDynamic(Direction.kReverse));
    this.joystick.start().and(this.joystick.y()).whileTrue(this.drivetrain.sysIdQuasistatic(Direction.kForward));
    this.joystick.start().and(this.joystick.x()).whileTrue(this.drivetrain.sysIdQuasistatic(Direction.kReverse));

    // Reset the field-centric heading on left bumper press.
    this.joystick.leftBumper().onTrue(this.drivetrain.runOnce(this.drivetrain::seedFieldCentric));

    this.drivetrain.registerTelemetry(this.logger::telemeterize);

    this.joystick.rightTrigger().onTrue(this.shooter.cycleSpeedCommand());
  }

  public Command getAutonomousCommand() {
    /* Run the path selected from the auto chooser */
    return this.autoChooser.getSelected();
  }
}