package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
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
  private Optional<Hub> hub = Optional.empty();

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
  public Optional<Hub> getHub() {
    if (hub.isEmpty()) {
      hub = getAlliance().map(a -> a == Alliance.Blue ? Hub.blue : Hub.red);
    }
    return hub;
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

  @Override
  public List<Translation2d> getFerryTargets() {
    return getAlliance().map(alliance -> alliance == Alliance.Blue ? blueFerryTargets : redFerryTargets)
        .orElse(List.of());
  }

  private double getTime() {
    return DriverStation.getMatchTime();
  }

  @Override
  public void periodic() {
    if (DEBUG) {
      SmartDashboard.putString("Game: My Alliance", alliance.toString());
      SmartDashboard.putString("Game: Auto Winner", autoWinner.toString());
      SmartDashboard.putString("Game: Hub", hub.toString());
    }
  }
}
