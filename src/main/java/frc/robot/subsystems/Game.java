package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Game information.
 */
public interface Game {
  public static record Hub(Translation2d position, Translation2d forward, List<Integer> tagIDs) {
    public static final Hub blue = new Hub(
        new Translation2d(Inches.of(182.11), Inches.of(158.84)),
        new Translation2d(1.0, 0.0),
        List.of(26, 21, 18));

    public static final Hub red = new Hub(
        new Translation2d(Inches.of(469.11),Inches.of(158.84)),
        new Translation2d(-1.0, 0.0),
        List.of(2, 5, 10));
  }

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
   * The current shift. Only valid for teleop period; empty if in autonomous
   * period.
   * 
   * @return The current shift, or empty if not in teleop period.
   */
  public Optional<Shift> getShift();

  public Pose2d getRedHubPos();

  public Pose2d getBlueHubPos();

  public Translation2d getHubPosition();

  public Translation2d getHubForward();

  /**
   * Our hub (according to our alliance).
   * 
   * @return Our hub, or empty if unknown.
   */
  public default Optional<Hub> getHub() {
    return getAlliance().map(alliance -> alliance == Alliance.Blue ? Hub.blue : Hub.red);
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
     * @return {@code true} if {@code time} is within the start and end time of this
     *         shift.
     */
    public boolean isActive(final double time) {
      return (time <= start && time >= end);
    }
  }
}
