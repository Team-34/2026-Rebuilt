package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Game extends SubsystemBase {
  private Optional<Shift> shift = Optional.empty();
  private Optional<Alliance> alliance = Optional.empty();

  public Optional<Alliance> getAlliance() {
    if (alliance.isEmpty()) {
      alliance = DriverStation.getAlliance();
    }
    return alliance;
  }

  public Optional<Alliance> getAutoWinner() {
    String gameData = DriverStation.getGameSpecificMessage();
    if (gameData.length() > 0) {
      switch (gameData.charAt(0)) {
        case 'B':
          // Blue case code
          return Optional.of(Alliance.Blue);
        case 'R':
          // Red case code
          return Optional.of(Alliance.Red);
        default:
          // This is corrupt data
          return Optional.of(Alliance.valueOf("NO ALLIANCE FOUND"));
      }
    }
    return Optional.of(Alliance.valueOf("NO ALLIANCE FOUND"));
  }

  public Optional<Shift> getShift() {
    return shift;
  }

  public double getTime() {
    return DriverStation.getMatchTime();
  }

  public static enum Shift {
    TRANSITION, ONE, TWO, THREE, FOUR, END;
  }
}
