package frc.robot.util;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Grams;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Newtons;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.stream.Stream;

import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

public class MathsTest {
  static Stream<Arguments> clamp_testCases() {
    return Stream.of(
      Arguments.of(Volts.of(-1), Volts.zero(), Volts.of(180), Volts.zero()),
      Arguments.of(Volts.of(181), Volts.zero(), Volts.of(180), Volts.of(180)),
      Arguments.of(Volts.of(57), Volts.zero(), Volts.of(180), Volts.of(57)),

      Arguments.of(Meters.of(-1), Meters.zero(), Meters.of(180), Meters.zero()),
      Arguments.of(Meters.of(181), Meters.zero(), Meters.of(180), Meters.of(180)),
      Arguments.of(Meters.of(57), Meters.zero(), Meters.of(180), Meters.of(57)),

      Arguments.of(Degrees.of(-1), Degrees.zero(), Degrees.of(180), Degrees.zero()),
      Arguments.of(Degrees.of(181), Degrees.zero(), Degrees.of(180), Degrees.of(180)),
      Arguments.of(Degrees.of(57), Degrees.zero(), Degrees.of(180), Degrees.of(57))
    );
  }

  @ParameterizedTest
  @MethodSource("clamp_testCases")
  public <T extends Comparable<? super T>> void clamp_constrainsInputToLimits(
    final T value, 
    final T low, 
    final T high, 
    final T expected
  ) {
    assertEquals(expected, Maths.clamp(value, low, high));
  }

  static Stream<Arguments> max_testCases() {
    return Stream.of(
      Arguments.of(Feet.of(45), Feet.of(99), Feet.of(99)),
      Arguments.of(Feet.of(3.55), Feet.of(-2.78), Feet.of(3.55)),

      Arguments.of(Grams.of(45), Grams.of(99), Grams.of(99)),
      Arguments.of(Grams.of(3.55), Grams.of(-2.78), Grams.of(3.55)),

      Arguments.of(Newtons.of(45), Newtons.of(99), Newtons.of(99)),
      Arguments.of(Newtons.of(3.55), Newtons.of(-2.78), Newtons.of(3.55))
    );
  }

  @ParameterizedTest
  @MethodSource("max_testCases")
  public <T extends Comparable<? super T>> void max_returnsTheGreaterOfTwoValues(
    final T a, 
    final T b, 
    final T expected
  ) {
    assertEquals(expected, Maths.max(a, b));
  }

  static Stream<Arguments> min_testCases() {
    return Stream.of(
      Arguments.of(Inches.of(45), Inches.of(99), Inches.of(45)),
      Arguments.of(Inches.of(3.55), Inches.of(-2.78), Inches.of(-2.78)),

      Arguments.of(Amps.of(45), Amps.of(99), Amps.of(45)),
      Arguments.of(Amps.of(3.55), Amps.of(-2.78), Amps.of(-2.78)),

      Arguments.of(Seconds.of(45), Seconds.of(99), Seconds.of(45)),
      Arguments.of(Seconds.of(3.55), Seconds.of(-2.78), Seconds.of(-2.78))
    );
  }

  @ParameterizedTest
  @MethodSource("min_testCases")
  public <T extends Comparable<? super T>> void min_returnsTheLesserOfTwoValues(
    final T a, 
    final T b, 
    final T expected
  ) {
    assertEquals(expected, Maths.min(a, b));
  }
}
