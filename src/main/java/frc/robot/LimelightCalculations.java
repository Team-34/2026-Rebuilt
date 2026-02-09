package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;

public class LimelightCalculations {
  final Pigeon2 gyro = new Pigeon2(10);

  /**
   * Returns the distance to a tag by utilizing the TZ value from a
   * LimelightHelpers array, and multiplying it by a constant value (of 65) to
   * convert it to inches.
   * 
   * @return The distance to a tag, in inches.
   */
  public static double getDistanceToTarget() {
    final int tzIndex = 2; // The index of the TZ value
    final int tzToInchesScalar = 65; // The scalar to convert TZ to inches, determined through testing

    final double[] targetPose_CameraSpace = LimelightHelpers.getTargetPose_CameraSpace("");

    return targetPose_CameraSpace[tzIndex] * tzToInchesScalar;
  }
}
