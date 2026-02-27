package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Game extends SubsystemBase {
  private Optional<Shift> shift = Optional.empty();
  private Optional<Alliance> alliance = Optional.empty();
  private Optional<Alliance> autoWinner = Optional.empty();

  public Optional<Alliance> getAlliance() {
    if (alliance.isEmpty()) {
      alliance = DriverStation.getAlliance();
    }
    return alliance;
  }

  public Optional<Alliance> getAutoWinner() {
    String gameData = DriverStation.getGameSpecificMessage();

    if (gameData.length() > 0) {
      this.autoWinner = switch (gameData.charAt(0)) {
        case 'B' -> Optional.of(Alliance.Blue);
        case 'R' -> Optional.of(Alliance.Red);
        default -> this.autoWinner = Optional.empty();
      };
    }
    
    return this.autoWinner;
  }

  public Optional<Shift> getShift() {
    return Optional.of(shift.get());
  }

  public double getTime() {
    return DriverStation.getMatchTime();
  }

  public static enum Shift {
    TRANSITION, ONE, TWO, THREE, FOUR, END;
  }
}
