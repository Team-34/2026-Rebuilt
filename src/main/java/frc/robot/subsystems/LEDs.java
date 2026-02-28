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

  /**
   * Constructs a new LEDs subsystem, 
   * setting the LED strip to an alliance color if available or if not, to solid green
   */
  public LEDs() {
    ledStrip.setLength(ledBuffer.getLength());
    solid(Color.kGreen);
    ledStrip.start();
    allianceColor(DriverStation.getAlliance());
  }

  /**
   * Sets the LED strip to a given solid color.
   * 
   * @param color the color to set the LED strip to.
   */
  public void solid(Color color) {
    LEDPattern pattern = LEDPattern.solid(color);
    pattern.applyTo(ledBuffer);
    ledStrip.setData(ledBuffer);
  }

  /**
   * Turns off the LED strip.
   */
  public void turnOff() {
    solid(Color.kBlack);
  }

  /**
   * Sets the LED strip to a solid red color.
   */
  public void red() {
    solid(Color.kRed);
  }

  /**
   * Sets the LED strip to a solid blue color.
   */
  public void blue() {
    solid(Color.kBlue);
  }

  /**
   * Sets the LED strip to the solid of the alliance if available.
   * 
   * Does not change the color if {@code alliance} is empty.
   * 
   * @param alliance this teams alliance.
   */
  public void allianceColor(Optional<DriverStation.Alliance> alliance) {
    alliance.ifPresent(a -> {
      if (a == Alliance.Red) {
        red();
      } else {
        blue();
      }
    });
  }
}