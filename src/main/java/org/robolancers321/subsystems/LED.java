/* (C) Robolancers 2024 */
package org.robolancers321.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.Random;
import java.util.TreeSet;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import org.robolancers321.util.VirtualSubsystem;

/*
 99.9% of this class is written by FRC Team 6328 Mechanical Advantage, huge thanks to them - Vincent Z
 Yeah your led thingy was too good to not steal thank you for making it - Alex T
*/

public class LED extends VirtualSubsystem {

  public static final int kLEDPWMPort = 0;
  public static final int kLEDStripLength = 25;

  private static final double kStrobeDuration = 0.2;
  private static final double kBreathDuration = 1.0;
  private static final double kRainbowCycleLength = 25.0;
  private static final double kRainbowDuration = 0.25;
  private static final double kWaveExponent = 0.4;
  private static final double kWaveCycleLength = 25.0;
  private static final double kWaveDuration = 3.0;
  private static final double kStripeDuration = 0.5;

  private static final int kMeteorSize = 5;
  private static final int kMeteorTrailDecay = 3;
  private static final int kCooling = 50;
  private static final int kSparking = 75;

  private static final boolean kMeteorRandomDecay = true;

  private static final LED instance = new LED(); // solely for VirtualSubsystem to run periodic()

  private static final Random rng = new Random();

  private final AddressableLED ledStrip;
  private final AddressableLEDBuffer ledBuffer;
  private static TreeSet<Signal> ledSignals;

  private Consumer<AddressableLEDBuffer> currPattern;

  private LED() {
    this.ledStrip = new AddressableLED(kLEDPWMPort);

    this.ledBuffer = new AddressableLEDBuffer(kLEDStripLength);

    ledStrip.setLength(ledBuffer.getLength());
    ledStrip.setData(ledBuffer);
    ledStrip.start();

    ledSignals =
        new TreeSet<Signal>(
            Comparator.comparingInt(Signal::priority)
                .reversed()); // sorted in descending order of priority
  }

  public record Signal(
      int priority, BooleanSupplier condition, Consumer<AddressableLEDBuffer> pattern) {}

  public static void registerSignal(
      int priority, BooleanSupplier condition, Consumer<AddressableLEDBuffer> pattern) {
    final var priorityIsAlreadyClaimed =
        ledSignals.stream().mapToInt(Signal::priority).anyMatch(p -> p == priority);

    if (priorityIsAlreadyClaimed) {
      DriverStation.reportWarning("Priority " + priority + " is already claimed", true);
      return;
    }

    ledSignals.add(new Signal(priority, condition, pattern));
  }

  public static Consumer<AddressableLEDBuffer> solid(Section section, Color color) {
    return buf -> solid(buf, section, color);
  }

  private static void solid(AddressableLEDBuffer buffer, Section section, Color color) {
    if (color == null) return;

    for (int i = section.start(); i < section.end(); i++) {
      buffer.setLED(i, color);
    }
  }

  public static Consumer<AddressableLEDBuffer> strobe(Section section, Color color) {
    return buf -> strobe(buf, section, color, kStrobeDuration);
  }

  private static void strobe(
      AddressableLEDBuffer buffer, Section section, Color color, double duration) {
    boolean on = ((Timer.getFPGATimestamp() % duration) / duration) > 0.5;
    solid(buffer, section, on ? color : Color.kBlack);
  }

  public static Consumer<AddressableLEDBuffer> breath(Section section, Color color1, Color color2) {
    return buf -> breath(buf, section, color1, color2);
  }

  private static void breath(
      AddressableLEDBuffer buffer, Section section, Color color1, Color color2) {
    breath(buffer, section, color1, color2, kBreathDuration);
  }

  private static void breath(
      AddressableLEDBuffer buffer, Section section, Color color1, Color color2, double duration) {
    breath(buffer, section, color1, color2, duration, Timer.getFPGATimestamp());
  }

  private static void breath(
      AddressableLEDBuffer buffer,
      Section section,
      Color color1,
      Color color2,
      double duration,
      double timestamp) {
    double x = ((timestamp % kBreathDuration) / kBreathDuration) * 2.0 * Math.PI;
    double ratio = (Math.sin(x) + 1.0) / 2.0;
    double red = (color1.red * (1 - ratio)) + (color2.red * ratio);
    double green = (color1.green * (1 - ratio)) + (color2.green * ratio);
    double blue = (color1.blue * (1 - ratio)) + (color2.blue * ratio);
    solid(buffer, section, new Color(red, green, blue));
  }

  public static Consumer<AddressableLEDBuffer> rainbow(Section section) {
    return buf -> rainbow(buf, section);
  }

  private static void rainbow(AddressableLEDBuffer buffer, Section section) {
    rainbow(buffer, section, kRainbowCycleLength, kRainbowDuration);
  }

  private static void rainbow(
      AddressableLEDBuffer buffer, Section section, double cycleLength, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() / duration) % 1.0)) * 180.0;
    double xDiffPerLed = 180.0 / cycleLength;
    for (int i = 0; i < section.end(); i++) {
      x += xDiffPerLed;
      x %= 180.0;
      if (i >= section.start()) {
        buffer.setHSV(i, (int) x, 255, 255);
      }
    }
  }

  public static Consumer<AddressableLEDBuffer> wave(Section section, Color color1, Color color2) {
    return buf -> wave(buf, section, color1, color2);
  }

  private static void wave(
      AddressableLEDBuffer buffer, Section section, Color color1, Color color2) {
    wave(buffer, section, color1, color2, kWaveCycleLength, kWaveDuration);
  }

  private static void wave(
      AddressableLEDBuffer buffer,
      Section section,
      Color color1,
      Color color2,
      double cycleLength,
      double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0 * Math.PI;
    double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
    for (int i = 0; i < section.end(); i++) {
      x += xDiffPerLed;
      if (i >= section.start()) {
        double ratio = (Math.pow(Math.sin(x), kWaveExponent) + 1.0) / 2.0;
        if (Double.isNaN(ratio)) {
          ratio = (-Math.pow(Math.sin(x + Math.PI), kWaveExponent) + 1.0) / 2.0;
        }
        if (Double.isNaN(ratio)) {
          ratio = 0.5;
        }
        double red = (color1.red * (1 - ratio)) + (color2.red * ratio);
        double green = (color1.green * (1 - ratio)) + (color2.green * ratio);
        double blue = (color1.blue * (1 - ratio)) + (color2.blue * ratio);
        buffer.setLED(i, new Color(red, green, blue));
      }
    }
  }

  public static Consumer<AddressableLEDBuffer> meteorRain(Section section, Color8Bit color) {
    return buf -> meteorRain(buf, section, color);
  }

  private static void meteorRain(AddressableLEDBuffer buffer, Section section, Color8Bit color) {
    meteorRain(buffer, section, color, kMeteorSize, kMeteorTrailDecay, kMeteorRandomDecay);
  }

  private static void meteorRain(
      AddressableLEDBuffer buffer,
      Section section,
      Color8Bit color,
      int meteorSize,
      int meteorTrailDecay,
      boolean meteorRandomDecay) {
    for (int i = 0; i < 2 * (section.end() - section.start()); i++) {

      // fade brightness all LEDs one step
      for (int j = 0; j < kLEDStripLength; j++) {
        if ((!meteorRandomDecay) || (rng.nextInt(10) > 5)) {
          fadeToBlack(buffer, j, meteorTrailDecay);
        }
      }

      // draw meteor
      for (int j = 0; j < meteorSize; j++) {
        if ((i - j < kLEDStripLength) && (i - j >= 0)) {
          buffer.setRGB(i - j, color.red, color.green, color.blue);
        }

        // speed delay for generating new meteor
        // ledStrip.setData(ledBuffer);
        // double speedDelay = 0.03;
        // new WaitCommand(speedDelay);

      }
    }
  }

  private static void fadeToBlack(AddressableLEDBuffer buffer, int ledNo, int fadeValue) {
    int r = buffer.getLED8Bit(ledNo).red;
    int g = buffer.getLED8Bit(ledNo).green;
    int b = buffer.getLED8Bit(ledNo).blue;

    r = (r <= 10) ? 0 : (int) r - (r * fadeValue / 256);
    g = (g <= 10) ? 0 : (int) g - (g * fadeValue / 256);
    b = (b <= 10) ? 0 : (int) b - (b * fadeValue / 256);

    buffer.setRGB(ledNo, r, g, b);
  }

  public static Consumer<AddressableLEDBuffer> fire() {
    return buf -> fire(buf, kCooling, kSparking);
  }

  private static void fire(AddressableLEDBuffer buffer, int cooling, int sparking) {

    int[] heat = new int[kLEDStripLength];
    int cooldown;

    for (int i = 0; i < kLEDStripLength; i++) {
      cooldown = rng.nextInt(0, ((cooling * 10) / kLEDStripLength) + 2);

      if (cooldown > heat[i]) {
        heat[i] = 0;
      } else {
        heat[i] = heat[i] - cooldown;
      }
    }

    // Step 2.  Heat from each cell drifts 'up' and diffuses a little
    for (int k = kLEDStripLength - 1; k >= 2; k--) {
      heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
    }

    // Step 3.  Randomly ignite new 'sparks' near the bottom
    if (rng.nextInt(255) < sparking) {
      int y = rng.nextInt(7);
      heat[y] = heat[y] + rng.nextInt(160, 255);
    }

    // Step 4.  Convert heat to LED colors
    for (int j = 0; j < kLEDStripLength; j++) {
      setPixelHeatColor(buffer, j, heat[j]);
    }
  }

  private static void setPixelHeatColor(AddressableLEDBuffer buffer, int pixel, int temperature) {

    // Scale ‘heat’ down from 0-255 to 0-191,
    // which can then be easily divided into three
    // equal ‘thirds’ of 64 units each.
    int spectrum = Math.round((temperature / 255) * 191);

    // calculate a value that ramps up from zero to 255 in each ‘third’ of the scale.
    int heatramp = spectrum & 63; // 0..63
    heatramp += 2; // scale up to 0..252

    // figure out which third of the spectrum we're in:
    if (spectrum > 128) { // hottest
      buffer.setRGB(pixel, 255, 255, heatramp);
    } else if (spectrum > 64) { // middle
      buffer.setRGB(pixel, 255, heatramp, 0);
    } else { // coolest
      buffer.setRGB(pixel, heatramp, 0, 0);
    }
  }

  public static Consumer<AddressableLEDBuffer> stripes(Section section, Color... colors) {
    return buf -> stripes(buf, section, List.of(colors));
  }

  private static void stripes(AddressableLEDBuffer buffer, Section section, List<Color> colors) {
    stripes(buffer, section, colors, section.end() - section.start(), kStripeDuration);
  }

  private static void stripes(
      AddressableLEDBuffer buffer,
      Section section,
      List<Color> colors,
      int length,
      double duration) {
    int offset = (int) (Timer.getFPGATimestamp() % duration / duration * length * colors.size());
    for (int i = section.start(); i < section.end(); i++) {
      int colorIndex =
          (int) (Math.floor((double) (i - offset) / length) + colors.size()) % colors.size();
      colorIndex = colors.size() - 1 - colorIndex;
      buffer.setLED(i, colors.get(colorIndex));
    }
  }

  public static enum Section {
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
    Optional<Consumer<AddressableLEDBuffer>> newPattern = Optional.empty();

    for (var signal : ledSignals) {
      if (signal.condition.getAsBoolean()) {
        newPattern = Optional.of(signal.pattern);
        break;
      }
    }

    newPattern.orElse(currPattern).accept(ledBuffer);

    ledStrip.setData(ledBuffer);
  }
}
