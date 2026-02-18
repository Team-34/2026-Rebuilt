package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase {

  Pigeon2 gyro = new Pigeon2(10);

  /**
   * Returns a value from the Targetpose_CameraSpace array.
   * @param index the index of the array to return as a double.
   * @return Returns data from the Targetpose_CameraSpace array. The index is as follows:
   * <ul>
   *  <li>0: Tx (degree measure left or right from the center crosshair)
   *  <li>1: Ty (degree measure up or down from the center crosshair)
   *  <li>2: Tz (distance from target, with each unit being 65 inches)
   *  <li>3: Pitch (static pitch value of the Limelight as set in the web API, in degrees)
   *  <li>4: Yaw (static yaw value of the Limelight as set in the web API, in degrees)
   *  <li>5: Roll (static roll value of the Limelight as set in the web API, in degrees)
   * </li>
   * </ul>
   */
  public double getTargetPose_CameraSpaceElement(int index) {
  double[] data = LimelightHelpers.getTargetPose_CameraSpace("");
  return data[index];
  }

  public void setPriorityTag(int tag) {
    LimelightHelpers.setPriorityTagID("", tag);
  }
  
  public double getDistanceToTarget() {
    final int tzToInchesScalar = 65;
    final int tzIndex = 2;
    return getTargetPose_CameraSpaceElement(tzIndex) * tzToInchesScalar;
  }
  
  public boolean isTargetValid() {
    return LimelightHelpers.getTV("");
  }

  public double getTX() {
    return getTargetPose_CameraSpaceElement(0);
  }
  
  public double getTY() {
    return getTargetPose_CameraSpaceElement(1);
  }

  public void updatePosition() {
    double yawInDegrees = gyro.getYaw().getValueAsDouble() % 360;
    LimelightHelpers.SetRobotOrientation("limelight", yawInDegrees, 0, 0, 0, 0, 0);
  }

  public boolean isTargetLocked(int tag) {
    final double TY_TOLERANCE = 0.5;
    final double TX_TOLERANCE = 0.5;
    boolean isCorrectTag = LimelightHelpers.getFiducialID("") == tag;
    boolean isWithinTolerance = MathUtil.isNear(0, getTX(), TX_TOLERANCE) && MathUtil.isNear(0, getTY(), TY_TOLERANCE);
    return isCorrectTag && isWithinTolerance;
  }

  public void periodic() {
    SmartDashboard.putNumber("Distance to Target", getDistanceToTarget());
    SmartDashboard.putNumber("Limelight Tx", getTargetPose_CameraSpaceElement(0));
    SmartDashboard.putNumber("Limelight Ty", getTargetPose_CameraSpaceElement(1));
    updatePosition();
  }
}
