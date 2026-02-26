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
    LEDPattern initColor = LEDPattern.solid(Color.kGreen);
    initColor.applyTo(ledBuffer);
    ledStrip.setData(ledBuffer);
    ledStrip.start();
    allianceColor(DriverStation.getAlliance());
  }

  public void turnOff() {
    LEDPattern black = LEDPattern.solid(Color.kBlack);
    black.applyTo(ledBuffer);
    ledStrip.setData(ledBuffer);
  }

  public void red() {
    LEDPattern red = LEDPattern.solid(Color.kRed);
    red.applyTo(ledBuffer);
    ledStrip.setData(ledBuffer);
  }

  public void blue(){
    LEDPattern blue = LEDPattern.solid(Color.kBlue);
    blue.applyTo(ledBuffer);
    ledStrip.setData(ledBuffer);
  }

  public void allianceColor(Optional<DriverStation.Alliance> alliance) {
    if(DriverStation.isEnabled()) {
      if(alliance.isPresent() && alliance.get() == Alliance.Red) {
        red();
      } else if(alliance.isPresent() && alliance.get() == Alliance.Blue) {
        blue();
      }
    }
  }
}