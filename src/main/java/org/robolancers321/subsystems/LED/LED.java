/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.LED;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.Random;
import java.util.TreeSet;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import org.robolancers321.util.MathUtils;
import org.robolancers321.util.VirtualSubsystem;

/*
 99.9% of this class is written by FRC Team 6328 Mechanical Advantage, huge thanks to them - Vincent Z
 Yeah your led thingy was too good to not steal thank you for making it - Alex T
 Mwahahahah meteor lmao - Matt P
*/

public class LED extends VirtualSubsystem {
  // TODO: move this into constants

  public static final int kLEDPWMPort = 9;
  public static final int kLEDStripLength = 69;

  private static final double kStrobeDuration = 0.2;
  private static final double kBreathDuration = 1.0;
  private static final double kRainbowCycleLength = 25.0;
  private static final double kRainbowDuration = 0.25;
  private static final double kWaveExponent = 0.4;
  private static final double kWaveCycleLength = 25.0;
  private static final double kWaveDuration = 3.0;
  private static final double kStripeDuration = 0.5;

  private static final int kWavefrontSpeed = 20;
  private static final int kFadeSpeed = 3000;
  private static final double kFadeProbability = 0.3;
  private static final int kWavefrontSeparation = 38;
  private static final int kWavefrontLength = 1;

  public static final rgb[] kDrivingMeteor = { new rgb(255, 50, 20)}; // 255, 40, 0
  public static final rgb[] kClimbingMeteor = {new rgb(0, 0, 255)};

  public static final rgb[] kNoteMeteorColors = {
    new rgb(250, 120, 20),
    // new rgb(250, 200, 0),
  };

  private static int colorIndex = 0;
  private static double wavefrontPosition = -1;

  private static rgb[] meteorBuf = new rgb[kLEDStripLength];

  private static final int kCooling = 30; // 20 - 100
  private static final int kSparking = 50; // 0-255
  private static final double kSpeedDelay = 1; // s
  private static final int kSparks = 3;
  private static final int kSparkHeight = 4;
  static int[] heat = new int[kLEDStripLength];

  private static final Random rng = new Random();

  public final AddressableLED ledStrip;
  private final AddressableLEDBuffer ledBuffer;
  private static TreeSet<Signal> ledSignals;

  private Consumer<AddressableLEDBuffer> currPattern = LED.meteorRain(0.20, kDrivingMeteor);

  public LED() {
    this.ledStrip = new AddressableLED(kLEDPWMPort);

    this.ledBuffer = new AddressableLEDBuffer(kLEDStripLength);

    ledStrip.setLength(ledBuffer.getLength());
    ledStrip.setData(ledBuffer);
    ledStrip.start();

    ledSignals =
        new TreeSet<Signal>(
            Comparator.comparingInt(Signal::priority)
                .reversed()); // sorted in descending order of priority

    Arrays.fill(heat, 0);
    Arrays.fill(meteorBuf, new rgb(0, 0, 0));
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

  public static Consumer<AddressableLEDBuffer> meteorRain(double dt, rgb[] meteorColors) {
    return buf -> meteorRain(buf, dt, meteorColors);
  }

  // private static void meteorRain(AddressableLEDBuffer buffer, double dt) {
  //   for (int i = 0; i < kLEDStripLength; i++) {
  //     if (Math.random() < kFadeProbability) fadeToBlack(buffer, i, dt);
  //   }

  //   wavefrontPosition += kWavefrontSpeed * dt;

  //   if (wavefrontPosition - kWavefrontLength >= kLEDStripLength)
  //     wavefrontPosition -= kWavefrontSeparation;

  //   var nthWavefrontPosition = wavefrontPosition;

  //   while (nthWavefrontPosition >= 0) {
  //     for (int i = 0; i < kWavefrontLength; i++) {
  //       if (nthWavefrontPosition - i >= 0 && nthWavefrontPosition - i < kLEDStripLength)
  //         buffer.setLED(nthWavefrontPosition - i, kMeteorColor);
  //     }
  //     nthWavefrontPosition -= kWavefrontSeparation;
  //   }
  // }

  // private static void fadeToBlack(AddressableLEDBuffer buffer, int index, double dt) {
  //   final Color8Bit currColor = buffer.getLED8Bit(index);
  //   final var decrement = kFadeSpeed * dt;
  //   final var r = (int) Math.max(currColor.red - decrement, 0);
  //   final var g = (int) Math.max(currColor.green - decrement, 0);
  //   final var b = (int) Math.max(currColor.blue - decrement, 0);

  //   buffer.setRGB(index, r, g, b);
  // }

  private static void meteorRain(AddressableLEDBuffer buffer, double dt, rgb[] meteorColors) {
    for (int i = 0; i < kLEDStripLength; i++) {
      if (rng.nextDouble() < kFadeProbability) {
        fadeToBlack(buffer, i, dt);
      }
    }

    wavefrontPosition += kWavefrontSpeed * dt;

    if (wavefrontPosition - kWavefrontLength >= kLEDStripLength) {
      wavefrontPosition -= kWavefrontSeparation;
      colorIndex++;
    }

    double nthWavefrontPosition = wavefrontPosition;

    int colorOffset = 0;

    while (nthWavefrontPosition >= 0) {
      for (int k = 0; k < kWavefrontLength; k++) {
        if ((int) Math.round(nthWavefrontPosition - k) >= 0
            && (int) Math.round(nthWavefrontPosition - k) < kLEDStripLength) {
          meteorBuf[(int) Math.round(nthWavefrontPosition - k)] =
              meteorColors[(colorIndex + colorOffset) % meteorColors.length];
        }
      }
      colorOffset++;
      nthWavefrontPosition -= kWavefrontSeparation;
    }

    for (int i = 0; i < kLEDStripLength; i++) {
      buffer.setRGB(i, meteorBuf[i].getRed(), meteorBuf[i].getGreen(), meteorBuf[i].getBlue());
    }
  }

  private static void fadeToBlack(AddressableLEDBuffer buffer, int ledNo, double dt) {
    Color8Bit color = buffer.getLED8Bit(ledNo);
    int r = (int) MathUtil.clamp(color.red - kFadeSpeed * dt * (Math.random() - 0.1) * (color.red > 0 ? 1 : 0), 0, 255);
    int g = (int) MathUtil.clamp(color.green - kFadeSpeed * dt * (Math.random() - 0.1) * (color.green > 0 ? 1 : 0), 0, 255);
    int b = (int) MathUtil.clamp(color.blue - kFadeSpeed * dt * (Math.random() - 0.1) * (color.blue > 0 ? 1 : 0), 0, 255);

    meteorBuf[ledNo] = new rgb(r, g, b);
  }

  private static class rgb {
    private int r;
    private int g;
    private int b;

    public rgb(int r, int g, int b) {
      this.r = r;
      this.g = g;
      this.b = b;
    }

    public int getRed() {
      return r;
    }

    public int getGreen() {
      return g;
    }

    public int getBlue() {
      return b;
    }
  }

  public static Consumer<AddressableLEDBuffer> fire() {
    return buf -> fire(buf, kCooling, kSparking, kSpeedDelay, kSparks, kSparkHeight);
  }

  private static void fire(
      AddressableLEDBuffer buffer,
      int cooling,
      int sparking,
      double speedDelay,
      int sparks,
      int sparkHeight) {

    // cool each index a little
    for (int i = 0; i < kLEDStripLength; i++) {
      heat[i] =
          MathUtil.clamp(heat[i] - rng.nextInt(0, ((cooling * 10) / kLEDStripLength) + 2), 0, 255);
    }

    // Next drift heat up and diffuse it a little but
    for (int i = 0; i < kLEDStripLength; i++)
      heat[i] =
          (heat[i] * 2
                  + heat[(i + 1) % kLEDStripLength] * 3
                  + heat[(i + 2) % kLEDStripLength] * 2
                  + heat[(i + 3) % kLEDStripLength] * 1)
              / 8;

    // Randomly ignite new sparks down in the flame kernel
    for (int i = 0; i < sparks; i++) {
      if (rng.nextInt(255) < sparking) {
        int y = kLEDStripLength - 1 - rng.nextInt(sparkHeight);
        heat[y] = MathUtil.clamp(heat[y] + rng.nextInt(160, 255), 0, 255);
      }
    }

    // set led colors
    for (int j = 0; j < kLEDStripLength; j++) {
      setPixelHeatColor(buffer, j, heat[j]);
    }

    // slow down fire activity
    new RunCommand(() -> new WaitCommand(speedDelay));
  }

  private static void setPixelHeatColor(AddressableLEDBuffer buffer, int pixel, int temperature) {
    if (temperature > 170) { // hottest: white
      buffer.setRGB(pixel, 255, 255, temperature);
    } else if (temperature > 60) { // middle: orange & yellow
      buffer.setRGB(pixel, 255, (int) (temperature), 0);
    } else { // lowest: red
      buffer.setRGB(pixel, temperature * 2, 0, 0);
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

    // newPattern.ifPresent(patt -> currPattern = patt);
    if (newPattern.isPresent()) currPattern = newPattern.get();

    currPattern.accept(ledBuffer);

    // meteorRain(ledBuffer, 0.02);
    // fire(ledBuffer, kCooling, kSparking, kSpeedDelay, kSparks, kSparkHeight);

    ledStrip.setData(ledBuffer);
  }
}
