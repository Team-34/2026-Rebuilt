package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

/**
 * A collection supplemental mathematical functions.
 */
public final class Maths {
  private Maths() {}

  /**
   * Returns the greater of two values.
   * 
   * @param <T> the type of values to determine the maximum of
   * @param a the first value
   * @param b the second value
   * @return {@code a} or {@code b}, whichever is greater
   */
  public static <T extends Comparable<? super T>> T max(final T a, final T b) {
    return a.compareTo(b) > 0 ? a : b;
  }

  /**
   * Returns the lesser of two values.
   * 
   * @param <T> the type of values to determine the minimum of
   * @param a the first value
   * @param b the second value
   * @return {@code a} or {@code b}, whichever is less
   */
  public static <T extends Comparable<? super T>> T min(final T a, final T b) {
    return a.compareTo(b) < 0 ? a : b;
  }

  /**
   * Clamps {@code value} to fit between {@code min} and {@code max}.
   * 
   * @param <T> the type of value to clamp and the type of the bounds of the range
   *            it is being clamped to
   * @param value value to clamp
   * @param low minimum allowed value
   * @param hight maximum allowed value
   * @return a clamped value that fits within the range [{@code min}, {@code max}]
   */
  public static <T extends Comparable<? super T>> T clamp(final T value, final T low, final T high) {
    return min(max(value, low), high);
  }

  /**
   * Normalizes an angle to the range [-180°, 180°].
   *
   * @param degrees Angle to normalize
   * @return angle normalized to the range [-180°, 180°]
   */
  public static double normalizeAngleNeg180To180(double degrees) {
    if (degrees < -180) {
      return  degrees += 360;
    } else if (degrees > 180) {
      return  degrees -= 360;
    } else {
      return degrees;
    }
  }

  /**
   * Normalizes an angle to the range [-180°, 180°].
   *
   * @param angle Angle to normalize
   * @return angle normalized to the range [-180°, 180°]
   */
  public static Angle normalizeNeg180To180(final Angle angle) {
    return Degrees.of(normalizeAngleNeg180To180(angle.in(Degrees)));
  }
}
