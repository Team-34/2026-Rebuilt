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
   * Called in RobotContainer constructor. Sets the length of the LED strip to the
   * length of the LED buffer, sets the LED strip to a solid green color, starts
   * the LED strip, and sets the LED strip to a solid red or blue color based on
   * the alliance color.
   */
  public LEDs() {
    ledStrip.setLength(ledBuffer.getLength());
    solid(Color.kGreen);
    ledStrip.start();
    allianceColor(DriverStation.getAlliance());
  }

  /**
   * Called in all solid color methods to set the LED strip to a solid color. This
   * helps reduce ammount of code duplication.
   * 
   * @param color
   */
  public void solid(Color color) {
    LEDPattern pattern = LEDPattern.solid(color);
    pattern.applyTo(ledBuffer);
    ledStrip.setData(ledBuffer);
  }

  /**
   * Called in RobotContainer.disable(). Turns off the LED strip by setting it to
   * a solid black color.
   */
  public void turnOff() {
    solid(Color.kBlack);
  }

  /**
   * Called in LEDs.allianceColor() Sets the LED strip to a solid red color.
   */
  public void red() {
    solid(Color.kRed);
  }

  /**
   * Called in LEDs.allianceColor() Sets the LED strip to a solid blue color.
   */
  public void blue() {
    solid(Color.kBlue);
  }

  /**
   * Called in LEDs constructor. Sets the LED strip to a solid red or blue color
   * based on the alliance color.
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