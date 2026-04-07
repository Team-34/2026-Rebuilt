package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase {
  private final Trigger robotPoseUpdatedTrigger = new Trigger(this::hasNewRobotPose);

  private Pose2d robotPose = Pose2d.kZero;

  private double lastRobotPoseTimestampSeconds = -1;
  private double currentRobotPoseTimestampSeconds = -1;

  private final Game game;
  private final Translation2d hubPos;

  public Vision(final Game game) {
    this.game = game;
    this.hubPos = game.getHubPosition();
  }

  public Trigger robotPoseUpdated() {
    return robotPoseUpdatedTrigger;
  }
  
  /**
   * Gets latest robot pose from the chassis camera.
   * @return latest robot pose from the chassis camera
   */
  public Pose2d getRobotPose() {
    return robotPose;
  }

  /**
   * Gets timestamp of the latest robot pose from the chassis camera.
   * @return timestamp of the latest robot pose from the chassis camera
   */
  public double getRobotPoseTimestamp() { 
    return currentRobotPoseTimestampSeconds;
  }

  public Optional<Angle> getAzimuthToHub() {
    final var tags = game.getHubTagIDs();
    final var fiducials = LimelightHelpers.getRawFiducials("");
    for (final var fiducial : fiducials) {
      if (tags.contains(fiducial.id)) {
        return Optional.of(Degrees.of(fiducial.txnc));
      }
    }
    return Optional.empty();
  }

  public Optional<Distance> getDistanceToHub() {
    return getAzimuthToHub().map(_az -> Meters.of(robotPose.getTranslation().getDistance(hubPos)));
  }

  private boolean hasNewRobotPose() {
    return currentRobotPoseTimestampSeconds != lastRobotPoseTimestampSeconds;
  }

  public double getTotalCameraLatency() {
    final var captureLatency = LimelightHelpers.getLatency_Capture("");
    final var pipelineLatency = LimelightHelpers.getLatency_Pipeline("");
    final var totalLatency = captureLatency + pipelineLatency;

    return totalLatency;
  }

  @Override
  public void periodic() {
    lastRobotPoseTimestampSeconds = currentRobotPoseTimestampSeconds;

    final var result = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-chassis");
    robotPose = result.pose;
    currentRobotPoseTimestampSeconds = result.timestampSeconds;

    SmartDashboard.putString("Distance to Hub", getDistanceToHub().toString());
    SmartDashboard.putString("bot pos - blue", LimelightHelpers.getBotPose2d_wpiBlue("limelight-chassis").toString());
    SmartDashboard.putString("bot pos - red", LimelightHelpers.getBotPose2d_wpiRed("limelight-chassis").toString());
  }
}
