package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Game.Hub;

public class Vision extends SubsystemBase {
  private static final boolean DEBUG = false;

  public static final String CHASSIS_LIMELIGHT_NAME = "limelight-chassis";
  public static final String TURRET_LIMELIGHT_NAME  = "limelight-turret";

  private final Game game;

  public String error;

  private Optional<Pose2d> robotPose = Optional.empty();

  private double lastRobotPoseTimestampSeconds = -1;
  private double currentRobotPoseTimestampSeconds = -1;


  public Vision(final Game game) {
    this.game = game;
  }

  public Trigger robotPoseUpdated() {
    SmartDashboard.putBoolean("Vision/robotPoseUpdated() ?", true);
    return new Trigger(this::hasNewRobotPose);
  }
  
  /**
   * Gets latest robot pose if it can be determined.
   * @return latest robot pose, or empty if indeterminate.
   */
  public Optional<Pose2d> getRobotPose() {
    return robotPose;
  }

  public Optional<Angle> getHubPose() {
    return getAzimuthToHub();
  }

  public Optional<Distance> getHubDistance() {
    return getDistanceToHub();
  }

  /**
   * Gets timestamp of the latest robot pose.
   * @return timestamp of the latest robot pose from the chassis camera
   */
  public double getRobotPoseTimestamp() { 
    return currentRobotPoseTimestampSeconds;
  }

  public Optional<Angle> getAzimuthToHub() {
    Translation2d delta = robotPose.get().getTranslation();
    if (delta == null) {
          error = "delta";
          return Optional.empty();
    }

    Rotation2d absoluteAngle = delta.getAngle();
    if (absoluteAngle == null) {
      error = "absolute Angle";
      return Optional.empty();
    }
    Rotation2d relativeAngle = absoluteAngle.minus(robotPose.get().getRotation());
    if (relativeAngle == null) {
      error = "relative Angle";
      return Optional.empty();
    }
    Angle angle = relativeAngle.getMeasure();
    if (angle == null) {
      error = "angle";
      return Optional.empty();
    }
    return Optional.of(angle);
  }

  public Optional<Distance> getDistanceToHub() {
    return robotPose.flatMap(pose -> game.getHub().map(hub -> {
      final var meters = pose.getTranslation().getDistance(hub.position());
      return Meters.of(meters);
    }));
  }

  private boolean hasNewRobotPose() {
    var result = currentRobotPoseTimestampSeconds != lastRobotPoseTimestampSeconds;
    if(result){
      SmartDashboard.putNumber("Vision/hasNewRobotPose() current robot pose timestamp", currentRobotPoseTimestampSeconds);
    }
    return result;
  }

  @Override
  public void periodic() {
    lastRobotPoseTimestampSeconds = currentRobotPoseTimestampSeconds;

    final var estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(CHASSIS_LIMELIGHT_NAME);
    robotPose = estimate.pose.equals(Pose2d.kZero) ?
        Optional.empty() : 
        Optional.of(estimate.pose);
    currentRobotPoseTimestampSeconds = estimate.timestampSeconds;

    if (DEBUG) {
      SmartDashboard.putString("Vision: Robot Pose", robotPose.toString());
      SmartDashboard.putNumber("Vision: Timestamp of Robot Pose (seconds)", currentRobotPoseTimestampSeconds);
      SmartDashboard.putString("Vision: Distance to Hub (in)", getDistanceToHub().map(distance -> distance.in(Inches)).toString());
      SmartDashboard.putString("Vision: getHubPose", getHubPose().toString());
      SmartDashboard.putString("Vision: getHubDistance", getHubDistance().toString());
      SmartDashboard.putString("Vision: error", error);
    }
  }
}
