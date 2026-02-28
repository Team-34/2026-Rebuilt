package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class StubGame implements Game {
  public Optional<Alliance> alliance = Optional.empty();
  public Optional<Alliance> autoWinner = Optional.empty();
  public Optional<Shift> shift = Optional.empty();

  @Override
  public Optional<Alliance> getAlliance() {
    return this.alliance;
  }

  @Override
  public Optional<Alliance> getAutoWinner() {
    return this.autoWinner;
  }

  @Override
  public Optional<Shift> getShift() {
    return this.shift;
  }

  public StubGame withAlliance(final Alliance alliance) {
    this.alliance = Optional.of(alliance);
    return this;
  }

  public StubGame withAutoWinnder(final Alliance winner) {
    this.autoWinner = Optional.of(winner);
    return this;
  }

  public StubGame withShift(final Shift shift) {
    this.shift = Optional.of(shift);
    return this;
  }
}
