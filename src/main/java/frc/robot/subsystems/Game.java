package frc.robot.subsystems;

import java.util.Collections;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Game information.
 */
public interface Game {
  static final List<Integer> redHubTagIDs = List.of(2, 5, 10);
  static final List<Integer> blueHubTagIDs = List.of(26, 21, 18);

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
   * The current shift. Only valid for teleop period; empty if in autonomous period.
   * 
   * @return The current shift, or empty if not in teleop period.
   */
  public Optional<Shift> getShift();

  public Pose2d getRedHubPos();

  public Pose2d getBlueHubPos();

  public Translation2d getHubPosition();

  public Translation2d getHubForward();

  /**
   * @return
   */
  public default List<Integer> getHubTagIDs() {
    return getAlliance()
      .map(alliance -> alliance == Alliance.Blue ? blueHubTagIDs : redHubTagIDs)
      .orElse(Collections.emptyList());
  }

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
