package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase {
  private static final boolean DEBUG = false;

  public static final String CHASSIS_LIMELIGHT_NAME = "limelight-chassis";
  public static final String TURRET_LIMELIGHT_NAME  = "limelight-turret";

  private final Trigger robotPoseUpdatedTrigger = new Trigger(this::hasNewRobotPose);

  private final Game game;

  private Optional<Pose2d> robotPose = Optional.empty();

  private double lastRobotPoseTimestampSeconds = -1;
  private double currentRobotPoseTimestampSeconds = -1;


  public Vision(final Game game) {
    this.game = game;
  }

  public Trigger robotPoseUpdated() {
    return robotPoseUpdatedTrigger;
  }
  
  /**
   * Gets latest robot pose if it can be determined.
   * @return latest robot pose, or empty if indeterminate.
   */
  public Optional<Pose2d> getRobotPose() {
    return robotPose;
  }

  /**
   * Gets timestamp of the latest robot pose.
   * @return timestamp of the latest robot pose from the chassis camera
   */
  public double getRobotPoseTimestamp() { 
    return currentRobotPoseTimestampSeconds;
  }

  public Optional<Angle> getAzimuthToHub() {
    return game.getHub().flatMap(hub -> {;
      final var fiducials = LimelightHelpers.getRawFiducials(TURRET_LIMELIGHT_NAME);
      for (final var fiducial : fiducials) {
        if (hub.tagIDs().contains(fiducial.id)) {
          return Optional.of(Degrees.of(fiducial.txnc));
        }
      }
      return Optional.empty();
    });
  }

  public Optional<Distance> getDistanceToHub() {
    return robotPose.flatMap(pose -> game.getHub().map(hub -> {
      final var meters = pose.getTranslation().getDistance(hub.center());
      return Meters.of(meters);
    }));
  }

  private boolean hasNewRobotPose() {
    return currentRobotPoseTimestampSeconds != lastRobotPoseTimestampSeconds;
  }

  public double getTotalCameraLatency() {
    final var captureLatency = LimelightHelpers.getLatency_Capture(CHASSIS_LIMELIGHT_NAME);
    final var pipelineLatency = LimelightHelpers.getLatency_Pipeline(CHASSIS_LIMELIGHT_NAME);
    final var totalLatency = captureLatency + pipelineLatency;

    return totalLatency;
  }

  @Override
  public void periodic() {
    lastRobotPoseTimestampSeconds = currentRobotPoseTimestampSeconds;

    final var estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(CHASSIS_LIMELIGHT_NAME);
    robotPose = estimate.pose.equals(Pose2d.kZero) ?
        Optional.empty() : 
        Optional.of(estimate.pose);
    currentRobotPoseTimestampSeconds = estimate.timestampSeconds;

    if (DEBUG) {
      SmartDashboard.putString("Vision: Robot Pose", robotPose.toString());
      SmartDashboard.putNumber("Vision: Timestamp of Robot Pose (seconds)", currentRobotPoseTimestampSeconds);
      SmartDashboard.putString("Vision: Distance to Hub (in)", getDistanceToHub().map(distance -> distance.in(Inches)).toString());
    }
  }
}
