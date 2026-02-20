package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  private final AddressableLED ledStrip = new AddressableLED(0);
  private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(10);
  private final LEDPattern rainbow = LEDPattern.rainbow(255, 128);
  
  LEDPattern red = LEDPattern.solid(Color.kRed);

  public LEDs() {
    ledStrip.setLength(ledBuffer.getLength());
    red.applyTo(ledBuffer);
    ledStrip.setData(ledBuffer);
    ledStrip.start();
  }
}