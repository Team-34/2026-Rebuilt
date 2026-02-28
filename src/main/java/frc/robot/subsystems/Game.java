package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Game information.
 */
public interface Game {
  /**
   * Our alliance.
   * 
   * @return Our alliance, or empty if unknown.
   */
  public Optional<Alliance> getAlliance();

  /**
   * The alliance that won the autonomous period.
   * 
   * @return The alliance that won the autonomous period, or empty if unknown.
   */
  public Optional<Alliance> getAutoWinner();

  /**
   * The curent shift. Only valid for teleop period; empty if in autonomous period.
   * 
   * @return The current shift, or empty if not in teleop period.
   */
  public Optional<Shift> getShift();

  /**
   * Teleop shifts with their start & end times (in seconds remaining in teleop).
   */
  public static enum Shift {
    TRANSITION(140, 130),
    ONE(130, 105),
    TWO(105, 80), 
    THREE(80, 55), 
    FOUR(55, 30), 
    END(30, 0);

    private final double start;
    private final double end;

    private Shift(final double start, final double end) {
      this.start = start;
      this.end = end;
    }

    /**
     * Is this shift active at the given the match time?
     * 
     * @param time Current match time in seconds left in the current period
     *             (Autonomous or Teleop)
     * @return {@code true} if {@code time} is within the start and end time of
     *         this shift.
     */
    public boolean isActive(final double time) {
      return (time <= start && time >= end);
    }
  }
}
