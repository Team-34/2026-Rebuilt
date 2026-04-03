// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.OperatorConstants;
import frc.robot.generated.TunerConstants;
//import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DriverStationGame;
import frc.robot.subsystems.Game;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer; 
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Game game = new DriverStationGame();
 // private final Climber climber = new Climber();
  private final Intake intake = new Intake();
  private final Spindexer spindexer = new Spindexer();
  private final Vision vision = new Vision(game);
  private final Turret turret = new Turret(game, vision);
  private final LEDs leds = new LEDs(game);
  private final Shooter shooter = new Shooter(vision);

  private final DriveCoefficient driveCoefficient = DriveCoefficient.FULL;

  private final SlewRateLimiter forwardFilter = new SlewRateLimiter(3.0);
  private final SlewRateLimiter turnFilter = new SlewRateLimiter(3.5);
  private final SlewRateLimiter rotateFilter = new SlewRateLimiter(1.8);

  enum DriveCoefficient {
    FULL(1.0), FARIS(0.5);

    public final double value;

    DriveCoefficient(final double coefficient) {
      this.value = coefficient;
    }

    public DriveCoefficient next() {
      return switch(this) {
        case FULL -> FARIS;
        case FARIS -> FULL;
      };
    }
  }
  
  // @formatter:off
  private final double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity


  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake shieldwall = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    

  private final Telemetry logger = new Telemetry(MaxSpeed);
  public final LimelightHelpers limelightHelpers = new LimelightHelpers();
  private final CommandXboxController joystick = new CommandXboxController(0);
  private final SendableChooser<Command> autoChooser;
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  /* Path follower */
  // private final SendableChooser<Command> autoChooser;
  // @formatter:on

  public RobotContainer() {
    // NamedCommands.registerCommand("Toggle Intake", intake.toggle());
    // NamedCommands.registerCommand("Cycle Shooter Speed", shooter.cycleSpeedCommand());
    //NamedCommands.registerCommand("RunIntake", Commands.parallel(intake.runIn(), intake.cycleDeploymentCommand()));
    NamedCommands.registerCommand("shooterAtIdle", shooter.runAtIdleCommand());
    NamedCommands.registerCommand("Run Spindexer", spindexer.spin());
    NamedCommands.registerCommand(
      "aimAndShoot", 
      Commands.parallel(
        Commands.parallel(shooter.shootByRPSCommand(), turret.pointAtHubCommand()).repeatedly(),
        Commands.waitSeconds(0.5).andThen(spindexer.spin())
      ).withTimeout(Seconds.of(5))
    );
    NamedCommands.registerCommand("Stop All", Commands.parallel(shooter.stop(), spindexer.stop(), turret.stop()));
    // NamedCommands.registerCommand("Aim At A.T", turret.pointAtHubCommand(0));
    // NamedCommands.registerCommand("Turret to 90", turret.swivelToCommand(Degree.of(90)));

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
          .withVelocityX(forwardFilter.calculate(joystick.getLeftY() * MaxSpeed )) // Drive forward with negative Y (forward)
          .withVelocityY(turnFilter.calculate(joystick.getLeftX() * MaxSpeed )) // Drive left with negative X (left)
          .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        )
    );

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    // Initiate shieldwall while both sticks are pressed.
    joystick.leftStick().and(joystick.rightStick()).whileTrue(drivetrain.applyRequest(() -> shieldwall));        
        
    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // Reset the field-centric heading on back button press.
    joystick.back().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    drivetrain.registerTelemetry(logger::telemeterize);

    // @formatter:on

    // ==== OUR SUBSYSTEM BINDINGS ====

    // joystick.y().onTrue(climber.toggleCommand());
    // joystick.povLeft().whileTrue(climber.extendCommand());
    // joystick.povRight().whileTrue(climber.retractCommand());

    joystick.a().onTrue(intake.runIn()).onFalse(intake.stop());
    joystick.b().onTrue(intake.runOut()).onFalse(intake.stop());
    //joystick.x().onTrue(intake.cycleDeploymentCommand());
    joystick.x().onTrue(intake.runByPower(0.15)).onFalse(intake.stop());

    //joystick.y().whileTrue(Commands.parallel(shooter.shootCommand(), turret.pointAtHubCommand()));
    joystick.y().whileTrue(Commands.parallel(shooter.shootByRPSCommand(), turret.pointAtHubCommand()));
    //joystick.y().onTrue(shooter.runFiringMotorByRPSCommand(RevolutionsPerSecond.of(47)));

    joystick.povUp().onTrue(shooter.increaseByRPSCommand()).onFalse(shooter.stop());
    joystick.povDown().onTrue(shooter.decreaseByRPSCommand()).onFalse(shooter.stop());

    joystick.rightTrigger().onTrue(shooter.cycleSpeedCommand());
    joystick.leftTrigger().onTrue(spindexer.spin()).onFalse(spindexer.stop());
    
    joystick.leftBumper().onTrue(turret.swivelByPowerCommand(0.1)).onFalse(turret.stop());
    joystick.rightBumper().onTrue(turret.swivelByPowerCommand(-0.1)).onFalse(turret.stop());
    
    
    joystick.povRight().onTrue(turret.swivelToCommand(Degree.of(90))); 
    // joystick.povLeft().onTrue(turret.findZeroCommand(0.1));

    //joystick.povUp().onTrue(shooter.setHoodPosition(1.0));
    //joystick.povDown().onTrue(shooter.setHoodPosition(0.0));
  }
    

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
   
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /**
   * Called in Robot.disabledInit().
   * Used by subsystems to disable what they need turned off when the robot is disabled.
   */
  public void disable() {
    leds.turnOff();
    CommandScheduler.getInstance().schedule(shooter.stop(), turret.stop(), spindexer.stop());
  }
  public void enable() {
    leds.allianceColor();
  }
}

