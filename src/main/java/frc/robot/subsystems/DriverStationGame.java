package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Game information, including cached information given by the Driver Station.
 */
public class DriverStationGame extends SubsystemBase implements Game {
  private static final boolean DEBUG = false;

  private Optional<Alliance> alliance = Optional.empty();
  private Optional<Alliance> autoWinner = Optional.empty();
  private List<Integer> hubTagIDs = List.of();

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

  @Override
  public List<Integer> getHubTagIDs() {
    if (hubTagIDs.isEmpty()) {
      alliance.ifPresent(a -> {
        hubTagIDs = a == Alliance.Blue ? blueHubTagIDs : redHubTagIDs;
      });
    }
    return hubTagIDs;
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

  /**
   * Determines which shift we're in. Only valid for teleop period, you will get an
   * Optional.empty() in autonomous.
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

  @Override
  public void periodic() {
    if (DEBUG) {
      SmartDashboard.putString("Game: My Alliance", alliance.toString());
      SmartDashboard.putString("Game: Auto Winner", autoWinner.toString());
      SmartDashboard.putString("Game: Hub IDs", hubTagIDs.toString());
    }
  }
}
