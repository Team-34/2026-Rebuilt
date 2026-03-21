package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import java.util.Map;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  private final AddressableLED ledStrip = new AddressableLED(1);
  private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(164); // length of the LED strip is 300
  private final AddressableLEDBufferView frontLedBufferView = ledBuffer.createView(0, 41); // one side of robot ~ 41 LEDs
  private final AddressableLEDBufferView backLedBufferView = ledBuffer.createView(42, 83).reversed();
  private final AddressableLEDBufferView leftLedBufferView = ledBuffer.createView(84, 123);
  private final AddressableLEDBufferView rightLedBufferView = ledBuffer.createView(124, 163).reversed();
  Color colorHere = new Color();
  private final Game game;
  
  /**
   * Constructs a new LEDs subsystem, 
   * setting the LED strip to an alliance color if available or if not, to solid green
   * 
   * @param game the game information to use for determining alliance color.
   */
  public LEDs(final Game game) {
    this.game = game;
    ledStrip.setLength(ledBuffer.getLength());
    solidColor(Color.kGreen);
    ledStrip.start();
  }

  /**
   * Sets the LED strip to a given solid color.
   * 
   * @param color the color to set the LED strip to.
   */
  public void solidColor(Color color) {
    LEDPattern solidPattern = LEDPattern.solid(color);
    solidPattern.applyTo(ledBuffer);
    ledStrip.setData(ledBuffer);
  }

/**
 * Sets the LED strip to a animated mask pattern.
 * 
 * @param color the color to set the LED strip to.
 */
public void animatedMask(Color color) {
    Map<Number, Color> maskSteps = Map.of(0, Color.kWhite,0.25, Color.kBlack, 0.5, Color.kWhite, 0.75, Color.kBlack, 1.0, Color.kWhite);
    LEDPattern base = LEDPattern.solid(color);
    LEDPattern mask = LEDPattern.steps(maskSteps).scrollAtRelativeSpeed(Percent.per(Second).of(50));
    LEDPattern maskPattern = base.mask(mask);
    maskPattern.applyTo(frontLedBufferView);
    maskPattern.applyTo(backLedBufferView);
    maskPattern.applyTo(leftLedBufferView);
    maskPattern.applyTo(rightLedBufferView);
    ledStrip.setData(ledBuffer);    
  }

  /**
   * Turns off the LED strip.
   */
  public void turnOff() {
    solidColor(Color.kBlack);
  }

  /**
   * Sets the LED strip to a solid red color.
   */
  public void red() {
    animatedMask(Color.kRed);
  }

  /**
   * Sets the LED strip to a solid blue color.
   */
  public void blue() {
    animatedMask(Color.kBlue);
  }

  /**
   * Sets the LED strip to the animated mask of the alliance if available.
   * 
   * Does not change the color if {@code alliance} is empty.
   * 
   */
  public void allianceColor() {
    game.getAlliance().ifPresent(a -> {
      if (a == Alliance.Blue) {
        blue();
      } else {
        red();
      }
    });
  }

  @Override
  public void periodic() {
  }
}