package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Strategy extends SubsystemBase {
  private final Game game = new Game();

  public boolean canScoreInHub() {
    var ourAlliance = game.getAlliance();
    var autoWinner = game.getAutoWinner();
    var activeShift = game.getShift().get().name();
    var areWeActive = autoWinner != ourAlliance;

    return switch(activeShift) {
      case "TRANSITION" -> true;
      case "ONE" -> areWeActive;
      case "TWO" -> !areWeActive;
      case "THREE" -> areWeActive;
      case "FOUR" -> !areWeActive;
      case "END" -> true;
      default -> false;
    };
  }
}
