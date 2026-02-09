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
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final Climber climber = new Climber();
  //private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();
  //private final Spindexer spindexer = new Spindexer();

  SlewRateLimiter ForwardFilter = new SlewRateLimiter(1.7);
  SlewRateLimiter TurnFilter = new SlewRateLimiter(1.7);
  SlewRateLimiter RotateFilter = new SlewRateLimiter(1.7);

  // @formatter:off
  private final double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandXboxController joystick = new CommandXboxController(0);
  private final SendableChooser<Command> autoChooser;
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  /* Path follower */
  // private final SendableChooser<Command> autoChooser;
  // @formatter:on

  public RobotContainer() {
    // this.autoChooser = AutoBuilder.buildAutoChooser("Tests");
    // SmartDashboard.putData("Auto Mode", this.autoChooser);

    this.configureBindings();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    // Warmup PathPlanner to avoid Java pauses
    FollowPathCommand.warmupCommand().schedule();
  }

  private void configureBindings() {
    // @formatter:off

    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive
            .withVelocityX(ForwardFilter.calculate(-joystick.getLeftY() * MaxSpeed)) // Drive forward with negative Y (forward)
            .withVelocityY(TurnFilter.calculate(-joystick.getLeftX() * MaxSpeed)) // Drive left with negative X (left)
            .withRotationalRate(RotateFilter.calculate(-joystick.getRightX() * MaxAngularRate)) // Drive counterclockwise with negative X (left)
        )
    );

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // Reset the field-centric heading on left bumper press.
    joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    drivetrain.registerTelemetry(logger::telemeterize);

    // @formatter:on

    // ==== OUR SUBSYSTEM BINDINGS ====

    // joystick.y().onTrue(climber.toggleCommand());
    // joystick.povLeft().whileTrue(climber.extendCommand());
    // joystick.povRight().whileTrue(climber.retractCommand());

    // joystick.a().whileTrue(intake.runIn()).onFalse(intake.stop());
    // joystick.b().whileTrue(intake.runOut()).onFalse(intake.stop());
    // joystick.x().onTrue(intake.toggle());

    joystick.rightTrigger().onTrue(shooter.cycleSpeedCommand());

  //   joystick.rightTrigger().onTrue(spindexer.spin()).onFalse(spindexer.stop());
  //   joystick.a().onTrue(spindexer.spin()).onFalse(spindexer.stop());
  //   joystick.b().onTrue(spindexer.spinReverse()).onFalse(spindexer.stop());
 }

  public Command getAutonomousCommand() {
    // Simple drive forward auton
    return autoChooser.getSelected();
  }
}