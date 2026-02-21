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
import edu.wpi.first.wpilibj2.command.Command;

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
  }

  public void turnOff() {
    // return runOnce(() -> {
    LEDPattern black = LEDPattern.solid(Color.kBlack);
    black.applyTo(ledBuffer);
    ledStrip.setData(ledBuffer);
    //ledStrip.start();
    // });
  }

  public void Red() {
    LEDPattern red = LEDPattern.solid(Color.kRed);
    red.applyTo(ledBuffer);
    ledStrip.setData(ledBuffer);
    //ledStrip.start();
  }

  public void rainbow(){

    m_scrollingRainbow.applyTo(ledBuffer);
    ledStrip.setData(ledBuffer);
  }

  @Override
  public void periodic() {
    //rainbow();
  }
}