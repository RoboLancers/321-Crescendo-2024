/* (C) Robolancers 2024 */
package org.robolancers321;


import org.robolancers321.subsystems.LED;
import org.robolancers321.subsystems.LED.LEDPattern;
import org.robolancers321.subsystems.LED.Section;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  private LED led = LED.getInstance();
  private CommandXboxController controller = new CommandXboxController(0);
  
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    LED.register(4, () -> controller.b().getAsBoolean(), LEDPattern.WAVE, Color.kBlue, Color.kRed); 
    LED.register(3, () -> controller.y().getAsBoolean(), LEDPattern.SOLID, Color.kGreen, null);
    LED.register(2, () -> controller.x().getAsBoolean(), LEDPattern.BREATH, Color.kBlue, Color.kRed); 
    LED.register(1, () -> controller.a().getAsBoolean(), LEDPattern.STROBE, Color.kYellow, null);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
