package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase {
  public static final String CHASSIS_LIMELIGHT_NAME = "limelight-chassis";
  public static final String TURRET_LIMELIGHT_NAME  = "limelight-turret";

  private static final Pose2d redHubPos = new Pose2d(Inches.of(469.11), Inches.of(158.84), Rotation2d.kZero);
  private static final Pose2d blueHubPos = new Pose2d(Inches.of(182.11), Inches.of(158.84), Rotation2d.kZero);

  private final Trigger robotPoseUpdatedTrigger = new Trigger(this::hasNewRobotPose);

  private final Game game;

  private Pose2d robotPose = new Pose2d();

  private double lastRobotPoseTimestampSeconds = -1;
  private double currentRobotPoseTimestampSeconds = -1;


  public Vision(final Game game) {
    this.game = game;
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
    final var fiducials = LimelightHelpers.getRawFiducials(TURRET_LIMELIGHT_NAME);
    for (final var fiducial : fiducials) {
      if (tags.contains(fiducial.id)) {
        return Optional.of(Degrees.of(fiducial.txnc));
      }
    }
    return Optional.empty();
  }

  public Optional<Distance> getDistanceToHub() {
    return game.getAlliance().flatMap(alliance -> getAzimuthToHub().map(_az -> {
      SmartDashboard.putString("Alliance from distance method", alliance.toString());
      final var botXInches = robotPose.getX();
      final var botYInches = robotPose.getY();
      SmartDashboard.putString("Bot Pos", robotPose.toString());
      if (alliance == Alliance.Blue) {
        final var hubXInches = blueHubPos.getMeasureX().in(Inches);
        final var hubYInches = blueHubPos.getMeasureY().in(Inches);
        final var deltaX = hubXInches - botXInches;
        final var deltaY = hubYInches - botYInches;
        return Inches.of(Math.sqrt((deltaX * deltaX) + (deltaY * deltaY)));
      } else {
        final var hubXInches = redHubPos.getMeasureX().in(Inches);
        final var hubYInches = redHubPos.getMeasureY().in(Inches);
        final var deltaX = hubXInches - botXInches;
        final var deltaY = hubYInches - botYInches;
        return Inches.of(Math.sqrt((deltaX * deltaX) + (deltaY * deltaY)));
      }
    }));
  }

  private boolean hasNewRobotPose() {
    return currentRobotPoseTimestampSeconds != lastRobotPoseTimestampSeconds;
  }

  @Override
  public void periodic() {
    lastRobotPoseTimestampSeconds = currentRobotPoseTimestampSeconds;

    final var result = LimelightHelpers.getBotPoseEstimate_wpiBlue(CHASSIS_LIMELIGHT_NAME);
    robotPose = result.pose;
    currentRobotPoseTimestampSeconds = result.timestampSeconds;

    SmartDashboard.putString("Vision: Robot Pose", robotPose.toString());
    SmartDashboard.putNumber("Vision: Timestamp of Robot Pose (seconds)", currentRobotPoseTimestampSeconds);
    SmartDashboard.putString("Vision: Distance to Hub", getDistanceToHub().toString());
  }
}
