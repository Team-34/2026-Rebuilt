package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Optional;
import java.util.stream.Stream;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.provider.Arguments;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class GameTest {

  static Stream<Arguments> isInAllianceZone_testCases() {
    return Stream.of();
  }

  @Test
  public void isInAllianceZone_indicatesWhetherATranslation2dIsInOurAllianceZone() {
    final var uut = new StubGame();
    uut.alliance = Optional.of(Alliance.Blue);
    final var pose = new Pose2d(
      new Translation2d(
        Game.Hub.blue.position().getMeasureX().minus(Inches.of(1)),
        Feet.of(10)),
      Rotation2d.kZero);
    assertTrue(uut.isInAllianceZone(pose));
  }
}
