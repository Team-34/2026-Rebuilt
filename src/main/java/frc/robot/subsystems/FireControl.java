package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FireControl extends SubsystemBase {
  private static final boolean DEBUG = true;

  private final CommandSwerveDrivetrain drivetrain;
  private final Vision vision;
  private final Game game;

  public FireControl(final CommandSwerveDrivetrain drivetrain, final Vision vision, final Game game) {
    this.drivetrain = drivetrain;
    this.vision = vision;
    this.game = game;
  }
  
  public Pose2d getRobotPose() {
    return drivetrain.getState().Pose;
  }

  public Optional<Angle> getAzimuthToHub() {
    return vision.getAzimuthToHub();
  }

  // public Translation2d getClosestTarget() {
  //   game.getFerryTargets();
  // }

  private static final Translation2d turretOffset = new Translation2d(Inches.of(3.5), Inches.of(-6));

  public Optional<Distance> getDistanceToHub() {
    return game.getHub().map(hub -> {
      final var meters = getRobotPose().getTranslation().plus(turretOffset).getDistance(hub.position());
      return Meters.of(meters);
    });
  }

  @Override
  public void periodic() {
    if(DEBUG){
      SmartDashboard.putString("Fire Ctrl: Hub Position", game.getHub().map(Game.Hub::position).toString());
      SmartDashboard.putString("Fire Ctrl: Robot Position", getRobotPose().toString());
      SmartDashboard.putString("Fire Ctrl: Hub Aimuth", getAzimuthToHub().map(Angle::toLongString).toString());
    }
  }
}
