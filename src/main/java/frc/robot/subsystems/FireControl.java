package frc.robot.subsystems;

import frc.firecontrol.FuelPhysicsSim;
import frc.firecontrol.ProjectileSimulator;
import frc.firecontrol.ShotCalculator;
import frc.firecontrol.ShotLUT;

import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Vision;

public class FireControl extends SubsystemBase {

  private final Vision vision;
  private final Game game;
  private final ShotCalculator shotCalc;
  private ShotCalculator.LaunchParameters shot;
  private final CommandSwerveDrivetrain commandSwerveDrivetrain = TunerConstants.createDrivetrain();

  public FireControl(Vision vision, Game game) {
    this.vision = vision;
    this.game = game;
    var latency = vision.getTotalCameraLatency();

    ShotCalculator.Config config = new ShotCalculator.Config();
    config.launcherOffsetX = Inches.of(-6.375).in(Meters); // how far forward the launcher is from robot center (m) -> Modified
    config.launcherOffsetY = Inches.of(3.25).in(Meters); // how far left, 0 if centered -> Modified
    config.phaseDelayMs = latency; // your vision pipeline latency -> Modified
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
    Translation2d hubCenter = game.getHubPosition(); // your target
    Translation2d hubForward = game.getHubForward(); // which way the hub faces
    var robotState = commandSwerveDrivetrain.getState();
    var robotSpeeds = robotState.Speeds;
    var robotPose = robotState.Pose;
    var robotGyro = commandSwerveDrivetrain.getPigeon2();

    ShotCalculator.ShotInputs inputs = new ShotCalculator.ShotInputs(
      robotPose,
      ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, robotPose.getRotation()),
      robotSpeeds,
      hubCenter, 
      hubForward, 
      0.9, // vision confidence, 0 to 1
      robotGyro.getPitch().getValue().in(Degrees), // pitch for tilt gate (0.0 if no gyro)
      robotGyro.getRoll().getValue().in(Degrees) // roll for tilt gate (0.0 if no gyro)
    );
    shot = shotCalc.calculate(inputs);
  }

  /**
   * @return {@code RPS} based on the calculation from the {@code ShotCalculator}.
   *         0 {@code RPS} if the shot is not valid and if the shot is not
   *         confident.
   */
  public AngularVelocity getCalculatedRPM() {
    if (shot.isValid() && shot.confidence() > 50) {
      return RPM.of(shot.rpm());
    }
    return RPM.of(0);
  }
}
