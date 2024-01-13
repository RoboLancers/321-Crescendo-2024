/* (C) Robolancers 2024 */
package org.robolancers321.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import java.util.List;
import org.robolancers321.util.VirtualSubsystem;

/*
 99.9% of this class is written by FRC Team 6328 Mechanical Advantage, huge thanks to them - Vincent Z
 Yeah your led thingy was too good to not steal thank you for making it - Alex T
*/

public class LEDs extends VirtualSubsystem {
  public static final int kLEDPWMPort = 0;
  public static final int kLEDStripLength = 20;
  private static final double breathDuration = 1.0;
  private static final double waveExponent = 0.4;

  private final AddressableLED ledStrip;
  private final AddressableLEDBuffer ledBuffer;

  public LEDs() {
    this.ledStrip = new AddressableLED(kLEDPWMPort);

    this.ledBuffer = new AddressableLEDBuffer(kLEDStripLength);

    ledStrip.setLength(ledBuffer.getLength());
    ledStrip.setData(ledBuffer);
    ledStrip.start();
  }

  private void solid(Section section, Color color) {
    if (color != null) {
      for (int i = section.start(); i < section.end(); i++) {
        ledBuffer.setLED(i, color);
      }
    }
  }

  private void strobe(Section section, Color color, double duration) {
    boolean on = ((Timer.getFPGATimestamp() % duration) / duration) > 0.5;
    solid(section, on ? color : Color.kBlack);
  }

  private void breath(Section section, Color c1, Color c2, double duration) {
    breath(section, c1, c2, duration, Timer.getFPGATimestamp());
  }

  private void breath(Section section, Color c1, Color c2, double duration, double timestamp) {
    double x = ((timestamp % breathDuration) / breathDuration) * 2.0 * Math.PI;
    double ratio = (Math.sin(x) + 1.0) / 2.0;
    double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
    double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
    double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
    solid(section, new Color(red, green, blue));
  }

  private void rainbow(Section section, double cycleLength, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() / duration) % 1.0)) * 180.0;
    double xDiffPerLed = 180.0 / cycleLength;
    for (int i = 0; i < section.end(); i++) {
      x += xDiffPerLed;
      x %= 180.0;
      if (i >= section.start()) {
        ledBuffer.setHSV(i, (int) x, 255, 255);
      }
    }
  }

  private void wave(Section section, Color c1, Color c2, double cycleLength, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0 * Math.PI;
    double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
    for (int i = 0; i < section.end(); i++) {
      x += xDiffPerLed;
      if (i >= section.start()) {
        double ratio = (Math.pow(Math.sin(x), waveExponent) + 1.0) / 2.0;
        if (Double.isNaN(ratio)) {
          ratio = (-Math.pow(Math.sin(x + Math.PI), waveExponent) + 1.0) / 2.0;
        }
        if (Double.isNaN(ratio)) {
          ratio = 0.5;
        }
        double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
        double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
        double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
        ledBuffer.setLED(i, new Color(red, green, blue));
      }
    }
  }

  private void stripes(Section section, List<Color> colors, int length, double duration) {
    int offset = (int) (Timer.getFPGATimestamp() % duration / duration * length * colors.size());
    for (int i = section.start(); i < section.end(); i++) {
      int colorIndex =
          (int) (Math.floor((double) (i - offset) / length) + colors.size()) % colors.size();
      colorIndex = colors.size() - 1 - colorIndex;
      ledBuffer.setLED(i, colors.get(colorIndex));
    }
  }

  private static enum Section {
    FULL;

    public int start() {
      return switch (this) {
        case FULL -> 0;
      };
    }

    public int end() {
      return switch (this) {
        case FULL -> kLEDStripLength;
      };
    }
  }

  @Override
  public void periodic() {
    solid(Section.FULL, Color.kRed);
    ledStrip.setData(ledBuffer);
  }
}
