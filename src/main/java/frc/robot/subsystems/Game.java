package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
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

  public static final Distance FIELD_WIDTH = Inches.of(317.69);
  public static final Distance FIELD_LENGTH = Inches.of(651.22);
  public static final Distance FERRY_TARGET_OFFSET = Feet.of(6);

  public static List<Translation2d> blueFerryTargets = List.of(
    new Translation2d(FERRY_TARGET_OFFSET, FERRY_TARGET_OFFSET), 
    new Translation2d(FERRY_TARGET_OFFSET, FIELD_WIDTH.minus(FERRY_TARGET_OFFSET))
  );
  public static List<Translation2d> redFerryTargets = List.of(
    new Translation2d(FIELD_LENGTH.minus(FERRY_TARGET_OFFSET), FERRY_TARGET_OFFSET), 
    new Translation2d(FIELD_LENGTH.minus(FERRY_TARGET_OFFSET), FIELD_WIDTH.minus(FERRY_TARGET_OFFSET))
  );

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

  /**
   * Our hub (according to our alliance).
   * 
   * @return Our hub, or empty if unknown.
   */
  public default Optional<Hub> getHub() {
    return getAlliance().map(alliance -> alliance == Alliance.Blue ? Hub.blue : Hub.red);
  }

  public default List<Translation2d> getFerryTargets() {
    return getAlliance()
    .map(alliance -> alliance == Alliance.Blue ? blueFerryTargets : redFerryTargets)
    .orElse(List.of());
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
