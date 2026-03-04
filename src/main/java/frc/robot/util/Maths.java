package frc.robot.util;

public class Maths {
  private Maths() {}

  public static <T extends Comparable<? super T>> T min(final T a, final T b) {
    return a.compareTo(b) < 0 ? a : b;
  }

  public static <T extends Comparable<? super T>> T clamp(final T value, final T low, final T high) {
    if (value.compareTo(low) < 0) {
      return low;
    } else if (value.compareTo(high) > 0) {
      return high;
    } else {
      return value;
    }
  }
}
