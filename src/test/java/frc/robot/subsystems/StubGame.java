package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
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

  public StubGame withAutoWinner(final Alliance winner) {
    this.autoWinner = Optional.of(winner);
    return this;
  }

  public StubGame withShift(final Shift shift) {
    this.shift = Optional.of(shift);
    return this;
  }

  @Override
  public Pose2d getRedHubPos() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getRedHubPos'");
  }

  @Override
  public Pose2d getBlueHubPos() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getBlueHubPos'");
  }

  @Override
  public Translation2d getHubPosition() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getHubPosition'");
  }

  @Override
  public Translation2d getHubForward() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getHubForward'");
  }
}
