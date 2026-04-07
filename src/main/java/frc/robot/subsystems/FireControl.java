package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Optional;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.firecontrol.ShotCalculator;

public class FireControl extends SubsystemBase {
  private final CommandSwerveDrivetrain drivetrain;
  private final Game game;
  private final ShotCalculator shotCalc;
  private ShotCalculator.LaunchParameters shot;

  public FireControl(final CommandSwerveDrivetrain drivetrain, final Game game, final Vision vision) {
    this.drivetrain = drivetrain;
    this.game = game;

    final ShotCalculator.Config config = new ShotCalculator.Config();
    config.launcherOffsetX = Inches.of(-6.375).in(Meters); // how far forward the launcher is from robot center (m) -> Modified
    config.launcherOffsetY = Inches.of(3.25).in(Meters); // how far left, 0 if centered -> Modified
    config.phaseDelayMs = vision.getTotalCameraLatency(); // your vision pipeline latency -> Modified
    config.mechLatencyMs = 20.0; // how long the mechanism takes to respond -> Default
    config.maxTiltDeg = 5.0; // suppress firing when chassis tilts past this (bumps/ramps) -> Default
    config.headingSpeedScalar = 1.0; // heading tolerance tightens with robot speed (0 to disable) -> Default
    config.headingReferenceDistance = 2.5; // heading tolerance scales with distance from hub -> Default

    shotCalc = new ShotCalculator(config);

    shotCalc.loadLUTEntry(Inches.of(87.25).in(Meters), RotationsPerSecond.of(50).in(RPM), Seconds.of(1.1).in(Seconds));
    shotCalc.loadLUTEntry(Inches.of(111.25).in(Meters), RotationsPerSecond.of(55).in(RPM), Seconds.of(1.1).in(Seconds));
    shotCalc.loadLUTEntry(Inches.of(135.25).in(Meters), RotationsPerSecond.of(59).in(RPM), Seconds.of(1.2).in(Seconds));
    shotCalc.loadLUTEntry(Inches.of(159.25).in(Meters), RotationsPerSecond.of(64).in(RPM), Seconds.of(1.4).in(Seconds));
    shotCalc.loadLUTEntry(Inches.of(183.25).in(Meters), RotationsPerSecond.of(69).in(RPM), Seconds.of(1.5).in(Seconds));
  }

  @Override
  public void periodic() {
    final var robotState = drivetrain.getState();
    final var robotSpeeds = robotState.Speeds;
    final var robotPose = robotState.Pose;
    final var robotGyro = drivetrain.getPigeon2();

    game.getHub().ifPresent(hub -> {
      final ShotCalculator.ShotInputs inputs = new ShotCalculator.ShotInputs(
        robotPose,
        ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, robotPose.getRotation()),
        robotSpeeds,
        hub.center(), 
        hub.forward(), 
        0.9, // vision confidence, 0 to 1
        robotGyro.getPitch().getValue().in(Degrees), // pitch for tilt gate (0.0 if no gyro)
        robotGyro.getRoll().getValue().in(Degrees) // roll for tilt gate (0.0 if no gyro)
      );
      shot = shotCalc.calculate(inputs);
    });
  }

  /**
   * @return {@code RPS} based on the calculation from the {@code ShotCalculator}.
   *         0 {@code RPS} if the shot is not valid and if the shot is not
   *         confident.
   */
  public Optional<AngularVelocity> getCalculatedRPM() {
    if (shot.isValid() && shot.confidence() > 50) {
      return Optional.of(RPM.of(shot.rpm()));
    }
    return Optional.empty();
  }
}
