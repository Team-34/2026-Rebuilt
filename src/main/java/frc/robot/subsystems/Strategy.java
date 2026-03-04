package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Strategy extends SubsystemBase {
  private final Game game;

  public Strategy(final Game game) {
    this.game = game;
  }

  public boolean canScoreInHub() {
    return game.getAlliance().flatMap(alliance ->
      game.getAutoWinner().flatMap(winner ->
        game.getShift().map(shift -> switch (shift) {
          case TRANSITION, END -> true;
          case TWO, FOUR       -> alliance == winner;
          case ONE, THREE      -> alliance != winner;
          default              -> false;
        })
      )
    ).orElse(false);
  }
}
