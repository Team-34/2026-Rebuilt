package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Game information, including cached information given by the Driver Station.
 */
public class Game extends SubsystemBase {
  private Optional<Alliance> alliance = Optional.empty();
  private Optional<Alliance> autoWinner = Optional.empty();

  /**
   * The allience given by the Driver Station.
   * @return The alliance. Can be empty if no alliance given by the Driver Station.
   */
  public Optional<Alliance> getAlliance() {
    if (alliance.isEmpty()) {
      alliance = DriverStation.getAlliance();
    }
    return alliance;
  }

  /**
   * To figure out who won the auto phase and who starts in shift 1.
   * 
   * @return Witch allaince won autonomous.
   */
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

  /**
   * Determines which shift we're in. Only valid for teleop period, you wil get a
   * Optional.empty() in autonomous.
   * 
   * @return current shift.
   */
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

  /**
   * The different shifts' start and end times.
   */
  public static enum Shift {
    TRANSITION(140, 130), ONE(130, 105), TWO(105, 80), THREE(80, 55), FOUR(55, 30), END(30, 0);

    private final double start;
    private final double end;

    private Shift(double start, double end) {
      this.start = start;
      this.end = end;
    }

    /**
     * Checks if any of the shifts are active based on the given match time.
     * 
     * @param time Current match time
     * @return True if {@code time} is within the start and end time of a shift.
     */
    public boolean isActive(double time) {
      return (time <= start && time >= end);
    }
  }
}
