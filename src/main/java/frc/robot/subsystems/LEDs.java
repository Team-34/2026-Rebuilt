package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  private final AddressableLED ledStrip = new AddressableLED(0);
  private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(300);

 
  public LEDs() {
    ledStrip.setLength(ledBuffer.getLength());
    solid(Color.kGreen);
    ledStrip.start();
    allianceColor(DriverStation.getAlliance());
  }

  public void solid(Color color) {
    LEDPattern pattern = LEDPattern.solid(color);
    pattern.applyTo(ledBuffer);
    ledStrip.setData(ledBuffer);
  }

  public void turnOff() {
    solid(Color.kBlack);
  }

  public void red() {
    solid(Color.kRed);
  }

  public void blue(){
    solid(Color.kBlue);
  }

  public void allianceColor(Optional<DriverStation.Alliance> alliance) {
    alliance.ifPresent(a -> {
      if (a == Alliance.Red)
        red();
      else 
        blue();
    });
  }
}