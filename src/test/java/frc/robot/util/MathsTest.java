package frc.robot.util;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
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
}
