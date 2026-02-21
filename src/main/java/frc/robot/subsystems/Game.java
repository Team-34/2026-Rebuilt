package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Game extends SubsystemBase {
  private Optional<Shift> shift = Optional.empty();
  private Optional<Alliance> alliance = Optional.empty();

  public Optional<Alliance> getAlliance() {
    if (alliance.isEmpty()) {
      alliance = DriverStation.getAlliance();
    }
    return alliance;
  }

  public Optional<Alliance> getAutoWinner() { 
  }  

  public Optional<Shift> getShift() {
    return shift;
  }

  public int getTime() {
  } 
  
  public static enum Shift {
    TRANSITION,
    ONE,
    TWO,
    THREE,
    FOUR,
    END;
  }
}
