package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static org.junit.jupiter.api.Assertions.assertEquals;
import java.util.stream.Stream;

import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

import edu.wpi.first.units.measure.Distance;

public class ShooterTest {
  static Stream<Arguments> distanceToFiringSpeed_testCases() {
    return Stream.of(
      Arguments.of(Inches.of(60), 0.475),
      Arguments.of(Inches.of(84), 0.55),
      Arguments.of(Inches.of(108), 0.6),
      Arguments.of(Inches.of(132), 0.64),
      Arguments.of(Inches.of(156), 0.7),
      Arguments.of(Inches.of(180), 0.75)
    );
  }

  @ParameterizedTest
  @MethodSource("distanceToFiringSpeed_testCases")
  public void distanceToFiringSpeed_testCases(
    final Distance distance,
    final double expected
  ) {
    assertEquals(expected, Shooter.distanceToFiringSpeed(distance), 0.01);
  }
}
