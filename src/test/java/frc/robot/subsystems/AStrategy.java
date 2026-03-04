package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.stream.Stream;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.Game.Shift;

public class AStrategy {
  @Test
  public void cannotScoreInHubWhenAllianceIsUnknown() {
    final var game = new StubGame();
    final var uut = new Strategy(game);

    assertFalse(uut.canScoreInHub());
  }

  @Test
  public void cannotScoreInHubWhenAutoWinnerIsUnknown() {
    final var game = new StubGame().withAlliance(Alliance.Blue);
    final var uut = new Strategy(game);

    assertFalse(uut.canScoreInHub());
  }

  @Test
  public void cannotScoreInHubWhenShiftIsUnknown() {
    final var game = new StubGame()
      .withAlliance(Alliance.Blue)
      .withAutoWinner(Alliance.Blue);

    final var uut = new Strategy(game);

    assertFalse(uut.canScoreInHub());
  }

  @Test
  public void canScoreInHubDuringTransition() {
    final var game = new StubGame()
      .withAlliance(Alliance.Blue)
      .withAutoWinner(Alliance.Red)
      .withShift(Shift.TRANSITION);

    final var uut = new Strategy(game);

    assertTrue(uut.canScoreInHub());
  }

  @Test
  public void canScoreInHubDuringEndGame() {
    final var game = new StubGame()
      .withAlliance(Alliance.Red)
      .withAutoWinner(Alliance.Blue)
      .withShift(Shift.END);

    final var uut = new Strategy(game);

    assertTrue(uut.canScoreInHub());
  }

  static Stream<Arguments> canOnlyScoreInHubDuringProperShift_testCases() {
    return Stream.of(
      Arguments.of(Alliance.Red, Alliance.Red, Shift.ONE,   false),
      Arguments.of(Alliance.Red, Alliance.Red, Shift.TWO,   true),
      Arguments.of(Alliance.Red, Alliance.Red, Shift.THREE, false),
      Arguments.of(Alliance.Red, Alliance.Red, Shift.FOUR,  true),

      Arguments.of(Alliance.Blue, Alliance.Red, Shift.ONE,   true),
      Arguments.of(Alliance.Blue, Alliance.Red, Shift.TWO,   false),
      Arguments.of(Alliance.Blue, Alliance.Red, Shift.THREE, true),
      Arguments.of(Alliance.Blue, Alliance.Red, Shift.FOUR,  false)
    );
  }

  @ParameterizedTest
  @MethodSource("canOnlyScoreInHubDuringProperShift_testCases")
  public void canOnlyScoreInHubDuringProperShift(
    final Alliance alliance,
    final Alliance winner,
    final Shift shift,
    final boolean expected
  ) {
    final var game = new StubGame()
      .withAlliance(alliance)
      .withAutoWinner(winner)
      .withShift(shift);

    final var uut = new Strategy(game);

    assertEquals(expected, uut.canScoreInHub());
  }
}
