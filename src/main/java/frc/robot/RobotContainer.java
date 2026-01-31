// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
 // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Intake intake = new Intake();
  
  private final Shooter shooter = new Shooter();

  private final Spindexer spindexer = new Spindexer();

  private final Telemetry logger = new Telemetry(this.MaxSpeed);

  private final CommandXboxController joystick = new CommandXboxController(0);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  /* Path follower */
  private final SendableChooser<Command> autoChooser;

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

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  SlewRateLimiter ForwardFilter = new SlewRateLimiter(1.7);
  SlewRateLimiter TurnFilter = new SlewRateLimiter(1.7);
  SlewRateLimiter RotateFilter = new SlewRateLimiter(1.7);

  public RobotContainer() {
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

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.a().whileTrue(intake.runIn()).onFalse(intake.stop());
    m_driverController.b().whileTrue(intake.runOut()).onFalse(intake.stop());
    m_driverController.x().whileTrue(intake.toggleIntakePneumatics());
    m_driverController.rightTrigger().onTrue(shooter.cycleSpeedCommand());
    m_driverController.rightTrigger().onTrue(spindexer.Spin()).onFalse(spindexer.Spinstop());
    m_driverController.a().onTrue(spindexer.Spin()).onFalse(spindexer.Spinstop());
    m_driverController.b().onTrue(spindexer.Spinreverse()).onFalse(spindexer.Spinstop());
  }

  public Command getAutonomousCommand() {
    /* Run the path selected from the auto chooser */
    return this.autoChooser.getSelected();
  }
}