package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static org.junit.jupiter.api.Assertions.assertEquals;
import java.util.Optional;
import java.util.stream.Stream;

import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class GameTest {

  static Stream<Arguments> isInAllianceZone_testCases() {
    final var blueGame = new StubGame();
    blueGame.alliance = Optional.of(Alliance.Blue);

    final var redGame = new StubGame();
    redGame.alliance = Optional.of(Alliance.Red);

    final var blueHubX = Game.Hub.blue.position().getMeasureX();
    final var redHubX  = Game.Hub.red.position().getMeasureX();

    final var justInsideBlueAllianceArea = new Pose2d(
      new Translation2d(blueHubX.minus(Inches.of(1)), Feet.of(10)),
      Rotation2d.kZero
    );
    final var justOutsideBlueAllianceArea = new Pose2d(
      new Translation2d(blueHubX.plus(Inches.of(1)), Feet.of(10)),
      Rotation2d.kZero
    );

    final var justInsideRedAllianceArea = new Pose2d(
      new Translation2d(redHubX.plus(Inches.of(1)), Feet.of(10)),
      Rotation2d.kZero
    );
    final var justOutsideRedAllianceArea = new Pose2d(
      new Translation2d(redHubX.minus(Inches.of(1)), Feet.of(10)),
      Rotation2d.kZero
    );
    return Stream.of(
      Arguments.of(blueGame, justInsideBlueAllianceArea,  true),
      Arguments.of(blueGame, justOutsideBlueAllianceArea, false),
      Arguments.of(blueGame, justOutsideRedAllianceArea,  false),
      Arguments.of(blueGame, justInsideRedAllianceArea,   false),

      Arguments.of(redGame, justInsideBlueAllianceArea,  false),
      Arguments.of(redGame, justOutsideBlueAllianceArea, false),
      Arguments.of(redGame, justOutsideRedAllianceArea,  false),
      Arguments.of(redGame, justInsideRedAllianceArea,   true)
    );
  }

  @ParameterizedTest
  @MethodSource("isInAllianceZone_testCases")
  public void isInAllianceZone_indicatesWhetherATranslation2dIsInOurAllianceZone(
    final Game uut,
    final Pose2d pose,
    final boolean expected
  ) {
    assertEquals(expected, uut.isInAllianceZone(pose));
  }
}
