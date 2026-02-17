package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  
  public Vision() {}

  Pigeon2 gyro = new Pigeon2(10);

  double gyroDegrees = gyro.getYaw().getValueAsDouble() % 360;

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
  public double getTPCSArray(int index) 
  {
  double[] data = LimelightHelpers.getTargetPose_CameraSpace("");
  return data[index];
  }

  public void setPriorityTag(int tag) 
  {
    LimelightHelpers.setPriorityTagID("", tag);
  }
  
  public double getDistanceToTarget()
  {
    final int tzToInchesScalar = 65;
    final int tzIndex = 2;
    return getTPCSArray(tzIndex) * tzToInchesScalar;
  }
  
  public boolean isTargetValid() 
  {
    return LimelightHelpers.getTV("");
  }

  public double getTX() {
    return getTPCSArray(0);
  }
  
  public double getTY() {
    return getTPCSArray(1);
  }

  public void updatePosition()
  {
    LimelightHelpers.SetRobotOrientation("limelight", gyroDegrees, 0, 0, 0, 0, 0);
  }

  public boolean isTargetLocked(int tag)
  {
    final double TY_TOLERANCE = 0.5;
    final double TX_TOLERANCE = 0.5;
    boolean isCorrectTag = LimelightHelpers.getFiducialID("") == tag;
    boolean isWithinTolerance = getTX() < TX_TOLERANCE && getTY() < TY_TOLERANCE;
    return isCorrectTag && isWithinTolerance;
  }

  public void periodic() {
    SmartDashboard.putNumber("Distance to Target", getDistanceToTarget());
    SmartDashboard.putNumber("Gyro Degrees", gyroDegrees);
    SmartDashboard.putNumber("Limelight Tx", getTPCSArray(0));
    SmartDashboard.putNumber("Limelight Ty", getTPCSArray(1));
    updatePosition();
  }
}
