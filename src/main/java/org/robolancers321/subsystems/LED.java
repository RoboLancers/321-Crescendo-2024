/* (C) Robolancers 2024 */
package org.robolancers321.subsystems;

import java.util.Comparator;
import java.util.Random;
import java.util.SortedSet;
import java.util.TreeSet;
import java.util.function.BooleanSupplier;

import org.robolancers321.util.VirtualSubsystem;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

/*
 99.9% of this class is written by FRC Team 6328 Mechanical Advantage, huge thanks to them - Vincent Z
 Yeah your led thingy was too good to not steal thank you for making it - Alex T
*/

public class LED extends VirtualSubsystem {

  private static LED instance;

  public static LED getInstance() {
    if (instance == null) {
      instance = new LED();
    }
    return instance;
  }

  private static final int kLEDPWMPort = 9;
  private static final int kLEDStripLength = 25;

  private static final double kStrobeDuration = 0.2;
  private static final double kBreathDuration = 1.0;
  private static final double kRainbowCycleLength = 25.0;
  private static final double kRainbowDuration = 0.25;
  private static final double kWaveExponent = 0.4;
  private static final double kWaveCycleLength = 25.0;
  private static final double kWaveDuration = 3.0;
  private static final double kStripeDuration = 0.5;
  private Random rng = new Random();


  private final AddressableLED ledStrip;
  private final AddressableLEDBuffer ledBuffer;
  private static SortedSet<LEDState> ledStateList;

  private Color color1;
  private Color color2;
  private LEDPattern pattern;
  

  // private final SendableChooser<LEDPattern> sendableChooser;

  private LED() {
    this.ledStrip = new AddressableLED(kLEDPWMPort);

    this.ledBuffer = new AddressableLEDBuffer(kLEDStripLength);

    ledStrip.setLength(ledBuffer.getLength());
    ledStrip.setData(ledBuffer);
    ledStrip.start();

    ledStateList = new TreeSet<LEDState>(Comparator.comparingInt(LEDState::getPriority));

    // this.sendableChooser = new SendableChooser<LEDPattern>();

    // sendableChooser.setDefaultOption("vincent", LEDPattern.SOLID);

    // sendableChooser.addOption("Solid", LEDPattern.SOLID);
    // sendableChooser.addOption("breath", LEDPattern.BREATH);
    // sendableChooser.addOption("wave", LEDPattern.WAVE);
    // sendableChooser.addOption("raindbow", LEDPattern.RAINBOW);
    // sendableChooser.addOption("stripes", LEDPattern.STRIPES);
    // sendableChooser.addOption("strobe", LEDPattern.STROBE);

    // SmartDashboard.putNumber("Red", 0);
    // SmartDashboard.putNumber("Blue", 0);
    // SmartDashboard.putNumber("Green", 0);
    // SmartDashboard.putData(sendableChooser);
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
    double x = ((timestamp % kBreathDuration) / kBreathDuration) * 2.0 * Math.PI;
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
        double ratio = (Math.pow(Math.sin(x), kWaveExponent) + 1.0) / 2.0;
        if (Double.isNaN(ratio)) {
          ratio = (-Math.pow(Math.sin(x + Math.PI), kWaveExponent) + 1.0) / 2.0;
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

  private void meteorRain(int red, int green, int blue, int meteorSize, int meteorTrailDecay, boolean meteorRandomDecay) { 
     for(int i = 0; i < kLEDStripLength+kLEDStripLength; i++) {
   
    // fade brightness all LEDs one step
      for(int j=0; j<kLEDStripLength; j++) {
        if( (!meteorRandomDecay) || (rng.nextInt(10) > 5) ) {
          fadeToBlack(j, meteorTrailDecay);        
        }
      }
    
      // draw meteor
      for(int j = 0; j < meteorSize; j++) {
        if( ( i-j < kLEDStripLength) && (i-j>=0) ) {
          ledBuffer.setRGB(i-j, red, green, blue);
      }

      //speed delay for generating new meteor 
      // ledStrip.setData(ledBuffer);
      // double speedDelay = 0.03;
      // new WaitCommand(speedDelay);

    }
 }
}

public void fadeToBlack(int ledNo, int fadeValue){
  int r = ledBuffer.getLED8Bit(ledNo).red;
  int g = ledBuffer.getLED8Bit(ledNo).green;
  int b = ledBuffer.getLED8Bit(ledNo).blue;

  r=(r<=10)? 0 : (int) r-(r*fadeValue/256);
  g=(g<=10)? 0 : (int) g-(g*fadeValue/256);
  b=(b<=10)? 0 : (int) b-(b*fadeValue/256);

  ledBuffer.setRGB(ledNo, r, g, b);
}

private void fire(int cooling, int sparking){

  int[] heat = new int[kLEDStripLength];
  int cooldown;
  
for( int i = 0; i < kLEDStripLength; i++) {
    cooldown = rng.nextInt(0, ((cooling * 10) / kLEDStripLength) + 2);
   
    if(cooldown>heat[i]) {
      heat[i]=0;
    } else {
      heat[i]=heat[i]-cooldown;
    }
  }
 
  // Step 2.  Heat from each cell drifts 'up' and diffuses a little
  for( int k= kLEDStripLength - 1; k >= 2; k--) {
    heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
  }
   
  // Step 3.  Randomly ignite new 'sparks' near the bottom
  if( rng.nextInt(255) < sparking ) {
    int y = rng.nextInt(7);
    heat[y] = heat[y] + rng.nextInt(160,255);
  }

  // Step 4.  Convert heat to LED colors
  for( int j = 0; j < kLEDStripLength; j++) {
    setPixelHeatColor(j, heat[j] );
  }
}

public void setPixelHeatColor(int pixel, int temperature){

// Scale ‘heat’ down from 0-255 to 0-191,
// which can then be easily divided into three
// equal ‘thirds’ of 64 units each.
  int spectrum = Math.round((temperature/255)*191);

// calculate a value that ramps up from zero to 255 in each ‘third’ of the scale.
  int heatramp = spectrum & 63; // 0..63
  heatramp += 2; // scale up to 0..252
 
  // figure out which third of the spectrum we're in:
  if( spectrum > 128) {                     // hottest
    ledBuffer.setRGB(pixel, 255, 255, heatramp);
  } else if( spectrum > 64) {             // middle
    ledBuffer.setRGB(pixel, 255, heatramp, 0);
  } else {                               // coolest
    ledBuffer.setRGB(pixel, heatramp, 0, 0);
  }
}


  // private void stripes(Section section, List<Color> colors, int length, double duration) {
  //   int offset = (int) (Timer.getFPGATimestamp() % duration / duration * length * colors.size());
  //   for (int i = section.start(); i < section.end(); i++) {
  //     int colorIndex =
  //         (int) (Math.floor((double) (i - offset) / length) + colors.size()) % colors.size();
  //     colorIndex = colors.size() - 1 - colorIndex;
  //     ledBuffer.setLED(i, colors.get(colorIndex));
  //   }
  // }


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

  public static enum LEDPattern {
    SOLID,
    STROBE,
    BREATH,
    RAINBOW,
    WAVE,
  }

  public static void register(int priority, BooleanSupplier condition, LEDPattern applyPattern, Color color1, Color color2){
    ledStateList.add(new LEDState(priority, condition, applyPattern, color1, color2));
  }

  public static class LEDState{
    int priority;
    BooleanSupplier condition;
    LEDPattern applyPattern;
    Color color1;
    Color color2;

    public LEDState(int priority, BooleanSupplier condition, LEDPattern applyPattern, Color color1, Color color2){
      this.priority = priority;
      this.condition = condition;
      this.applyPattern = applyPattern;
      this.color1 = color1;
      this.color2 = color2;
    }

    public int getPriority(){
      return priority;
    }

    public BooleanSupplier isCondition(){
      return condition;
    } 

    public LEDPattern getApplyPattern(){
      return applyPattern;
    }

    public Color getColor1(){
      return color1;
    }
    
    public Color getColor2(){
      return color2;
    }
  }


  @Override
  public void periodic() {

    //test LEDs

    // double r = SmartDashboard.getNumber("Red", 0);
    // double g = SmartDashboard.getNumber("Green", 0);
    // double b = SmartDashboard.getNumber("Blue", 0);

    // Color color = new Color(r, g, b);

    // SmartDashboard.putString("sendableChooser", sendableChooser.getSelected().toString());

     
    // switch(sendableChooser.getSelected()){
    //   case SOLID -> solid(Section.FULL, color);
    //   case STROBE -> strobe(Section.FULL, color, strobeDuration);
    //   case STRIPES -> stripes(Section.FULL, List.of(Color.kRed, Color.kYellow, Color.kWhite), kLEDStripLength, stripeDuration);
    //   case BREATH -> breath(Section.FULL, color, Color.kAliceBlue, breathDuration);
    //   case RAINBOW -> rainbow(Section.FULL, rainbowCycleLength, rainbowDuration);
    //   case WAVE -> wave(Section.FULL, color, Color.kRed, waveCycleLength, waveDuration);
    // }


    for(LEDState state: ledStateList){
      if(state.isCondition().getAsBoolean()){

        color1 = state.getColor1();
        color2 = state.getColor2();
        pattern = state.getApplyPattern();

        break;
      }
    }

    switch(pattern){
      case SOLID -> solid(Section.FULL, color1);
      case STROBE -> strobe(Section.FULL, color1, kStrobeDuration);
      case BREATH -> breath(Section.FULL, color1, color2, kBreathDuration);
      case RAINBOW -> rainbow(Section.FULL, kRainbowCycleLength, kRainbowDuration);
      case WAVE -> wave(Section.FULL, color1, color2, kWaveCycleLength, kWaveDuration);
    }

    ledStrip.setData(ledBuffer);

    
  }
}
