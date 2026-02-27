package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Game extends SubsystemBase {
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
        default -> Optional.empty();
      };
    }

    return this.autoWinner;
  }

  public Optional<Shift> getShift() {
    if (DriverStation.isTeleop()) {
      for (var shift : Shift.values()) {
        if (shift.isActive(getTime())) {
          return Optional.of(shift);
        }
      }
    }

    return Optional.empty();
  }

  private double getTime() {
    return DriverStation.getMatchTime();
  }

  public static enum Shift {
    TRANSITION(140, 130), ONE(130, 105), TWO(105, 80), THREE(80, 55), FOUR(55, 30), END(30, 0);

    private final double start;
    private final double end;

    private Shift(double start, double end) {
      this.start = start;
      this.end = end;
    }

    public double getStartTime() {
      return start;
    }

    public double getEndTime() {
      return end;
    }

    public boolean isActive(double time) {
      return (time <= start && time >= end);
    }

  }

}
