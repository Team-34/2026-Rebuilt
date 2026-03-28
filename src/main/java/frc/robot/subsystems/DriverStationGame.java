package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Game information, including cached information given by the Driver Station.
 */
public class DriverStationGame extends SubsystemBase implements Game {
  private Optional<Alliance> alliance = Optional.empty();
  private Optional<Alliance> autoWinner = Optional.empty();

  private final Pose2d redHubPos = new Pose2d(Inches.of(469.11), Inches.of(158.84), Rotation2d.kZero);
  private final Pose2d blueHubPos = new Pose2d(Inches.of(182.11), Inches.of(158.84), Rotation2d.kZero);

  public static record Hub(Translation2d position, Translation2d forward) {}

  /**
   * The alliance given by the Driver Station.
   * 
   * @return The alliance. Can be empty if no alliance given by the Driver
   *         Station.
   */
  @Override
  public Optional<Alliance> getAlliance() {
    if (alliance.isEmpty()) {
      alliance = DriverStation.getAlliance();
    }
    return alliance;
  }

  /**
   * To figure out who won the auto phase and who starts in shift 1.
   * 
   * @return Which alliance won autonomous.
   */
  @Override
  public Optional<Alliance> getAutoWinner() {
    final String gameData = DriverStation.getGameSpecificMessage();

    if (gameData.length() > 0) {
      this.autoWinner = switch (gameData.charAt(0)) {
        case 'B' -> Optional.of(Alliance.Blue);
        case 'R' -> Optional.of(Alliance.Red);
        default -> Optional.empty();
      };
    }

    return this.autoWinner;
  }

  public Translation2d getHubPosition() {
    return getAlliance().map(alliance -> {
      if (alliance == Alliance.Blue) {
        return blueHubPos.getTranslation();
      } else {
        return redHubPos.getTranslation();
      }
    }).orElse(Translation2d.kZero);
  }

  public Translation2d getHubForward() {
    return getAlliance().map(alliance -> {
      if (alliance == Alliance.Blue) {
        return new Translation2d(1.0, 0.0);
      } else {
        return new Translation2d(-1.0, 0.0);
      }
    }).orElse(Translation2d.kZero);
  }

  public Pose2d getRedHubPos() {
    return redHubPos;
  }

  public Pose2d getBlueHubPos() {
    return blueHubPos;
  }

  /**
   * Determines which shift we're in. Only valid for teleop period, you will get
   * an Optional.empty() in autonomous.
   * 
   * @return Current shift.
   */
  @Override
  public Optional<Shift> getShift() {
    if (DriverStation.isTeleop()) {
      for (final var shift : Shift.values()) {
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
}
