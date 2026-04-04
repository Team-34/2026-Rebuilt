package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.MathUtil;
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

  Pigeon2 gyro = new Pigeon2(10);

  private final Trigger robotPoseUpdatedTrigger = new Trigger(this::hasNewRobotPose);

  private final Pose2d redHubPos = new Pose2d(Inches.of(469.11), Inches.of(158.84), Rotation2d.kZero);
  private final Pose2d blueHubPos = new Pose2d(Inches.of(182.11), Inches.of(158.84), Rotation2d.kZero);

  private Pose2d robotPose = new Pose2d();

  private double lastRobotPoseTimestampSeconds = -1;
  private double currentRobotPoseTimestampSeconds = -1;

  private final Game game;

  public Vision(final Game game) {
    this.game = game;
  }

  /**
   * Returns a value from the Targetpose_CameraSpace array.
   * 
   * @param index the index of the array to return as a double.
   * @return Returns data from the Targetpose_CameraSpace array. The index is as
   *         follows:
   *         <ul>
   *         <li>0: Tx (degree measure left or right from the center crosshair)
   *         <li>1: Ty (degree measure up or down from the center crosshair)
   *         <li>2: Tz (distance from target, with each unit being 65 inches)
   *         <li>3: Pitch (static pitch value of the Limelight as set in the web
   *         API, in degrees)
   *         <li>4: Yaw (static yaw value of the Limelight as set in the web API,
   *         in degrees)
   *         <li>5: Roll (static roll value of the Limelight as set in the web
   *         API, in degrees)</li>
   *         </ul>
   */
  public double getTargetPose_CameraSpaceArrayElement(final int index) {
    final double[] data = LimelightHelpers.getTargetPose_CameraSpace("");
    return data[index];
  }

  public void setPriorityTag(final int tag) {
    LimelightHelpers.setPriorityTagID("", tag);
  }

  // public double getDistanceToTarget() {
  // final int tzToInchesScalar = 65;
  // final int tzIndex = 2;
  // return getTargetPose_CameraSpaceArrayElement(tzIndex) * tzToInchesScalar;
  // }

  public boolean isTargetValid() {
    return LimelightHelpers.getTV("");
  }

  public Trigger robotPoseUpdated() {
    return robotPoseUpdatedTrigger;
  }
  
  public Pose2d getRobotPose() {
    return robotPose;
  }

  public double getRobotPoseTimestamp() { 
    return currentRobotPoseTimestampSeconds;
  }

  public double getTX() {
    return getTargetPose_CameraSpaceArrayElement(0);
  }

  public double getTY() {
    return getTargetPose_CameraSpaceArrayElement(1);
  }

  // public void updatePosition() {
  // double yawInDegrees = gyro.getYaw().getValueAsDouble() % 360;
  // LimelightHelpers.SetRobotOrientation("limelight", yawInDegrees, 0, 0, 0, 0,
  // 0);
  // }

  public boolean isTargetLocked(final int tag) {
    final double TY_TOLERANCE = 0.5;
    final double TX_TOLERANCE = 0.5;
    final boolean isCorrectTag = LimelightHelpers.getFiducialID("") == tag;
    final boolean isWithinTolerance = MathUtil.isNear(0, getTX(), TX_TOLERANCE)
        && MathUtil.isNear(0, getTY(), TY_TOLERANCE);
    return isCorrectTag && isWithinTolerance;
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
    final var result = LimelightHelpers.getBotPoseEstimate_wpiBlue("");
    robotPose = result.pose;
    currentRobotPoseTimestampSeconds = result.timestampSeconds;

    SmartDashboard.putString("Distance to Hub", getDistanceToHub().toString());
    // SmartDashboard.putNumber("Limelight Tx",
    // getTargetPose_CameraSpaceArrayElement(0));
    // SmartDashboard.putNumber("Limelight Ty",
    // getTargetPose_CameraSpaceArrayElement(1));
    SmartDashboard.putString("bot pos - blue", LimelightHelpers.getBotPose2d_wpiBlue("").toString());
    SmartDashboard.putString("bot pos - red", LimelightHelpers.getBotPose2d_wpiRed("").toString());
    // updatePosition();
  }
}
