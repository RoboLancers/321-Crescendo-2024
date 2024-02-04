/* (C) Robolancers 2024 */
package org.robolancers321;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import org.robolancers321.subsystems.LED;
import org.robolancers321.subsystems.LED.Section;

public class RobotContainer {
  private CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // example usage
    LED.registerSignal(5, controller.leftBumper(), LED.rainbow(Section.FULL));
    LED.registerSignal(4, controller.b(), LED.wave(Section.FULL, Color.kBlue, Color.kRed));
    LED.registerSignal(3, controller.y(), LED.solid(Section.FULL, Color.kGreen));
    LED.registerSignal(2, controller.x(), LED.breath(Section.FULL, Color.kBlue, Color.kRed));
    LED.registerSignal(1, controller.a(), LED.strobe(Section.FULL, Color.kYellow));

    RobotModeTriggers.teleop()
        .onTrue(
            Commands.runOnce(
                () ->
                    // 0 priority & constant true condition effectively acts as a default pattern
                    LED.registerSignal(
                        0,
                        () -> true,
                        LED.solid(
                            Section.FULL,
                            DriverStation.getAlliance().get() == Alliance.Red
                                ? Color.kRed
                                : Color.kBlue))));

    LED.registerSignal(
        6,
        controller.rightBumper(),
        buf -> {
          // iterate through buffer and apply colors as desired
        });
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
