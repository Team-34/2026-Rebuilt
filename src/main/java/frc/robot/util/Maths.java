package frc.robot.util;

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
}
