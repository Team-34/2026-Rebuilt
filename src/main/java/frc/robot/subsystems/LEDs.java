package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class LEDs extends SubsystemBase {
  private final AddressableLED ledStrip = new AddressableLED(0);
  private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(300);

  LEDPattern rainbow = LEDPattern.rainbow(255, 128);
  Distance kLedSpacing = Meters.of(1 / 300.0);
  LEDPattern m_scrollingRainbow = rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(0.1), kLedSpacing).atBrightness(Percent.of(10));

  public LEDs() {
    ledStrip.setLength(ledBuffer.getLength());
    LEDPattern initColor = LEDPattern.solid(Color.kGreen);
    initColor.applyTo(ledBuffer);
    ledStrip.setData(ledBuffer);
    ledStrip.start();
    allianceColor();
  }

  public void turnOff() {
    LEDPattern black = LEDPattern.solid(Color.kBlack);
    black.applyTo(ledBuffer);
    ledStrip.setData(ledBuffer);
  }

  public void Red() {
    LEDPattern red = LEDPattern.solid(Color.kRed);
    red.applyTo(ledBuffer);
    ledStrip.setData(ledBuffer);
  }

  public void Blue(){
    LEDPattern blue = LEDPattern.solid(Color.kBlue);
    blue.applyTo(ledBuffer);
    ledStrip.setData(ledBuffer);
  }

  public void rainbow(){
    m_scrollingRainbow.applyTo(ledBuffer);
    ledStrip.setData(ledBuffer);
  }

  public void allianceColor() {
    if(DriverStation.isEnabled()) {
      var ally = DriverStation.getAlliance().get();
      if(ally == Alliance.Red) {
        Red();
      } else if(ally == Alliance.Blue) {
        Blue();
      }
    }
  }

  @Override
  public void periodic(){

  }
}