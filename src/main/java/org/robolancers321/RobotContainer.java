/* (C) Robolancers 2024 */
package org.robolancers321;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.robolancers321.subsystems.LED;

public class RobotContainer {
  LED led = new LED();
  // AddressableLEDSim ledSim = new AddressableLEDSim(led.ledStrip);
  // private CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // example usage
    // LED.registerSignal(4, controller.b(), LED.wave(Section.FULL, Color.kRed, Color.kWhite));
    // LED.registerSignal(3, controller.y(), LED.wave(Section.FULL, Color.kYellow, Color.kWhite));
    // LED.registerSignal(2, controller.x(), LED.wave(Section.FULL, Color.kBlue, Color.kWhite));
    // LED.registerSignal(1, controller.a(), LED.wave(Section.FULL, Color.kGreen, Color.kWhite));

    // RobotModeTriggers.teleop()
    //     .onTrue(
    //         Commands.runOnce(
    //             () ->
    //                 // 0 priority & constant true condition effectively acts as a default pattern
    //                 LED.registerSignal(
    //                     0,
    //                     () -> true,
    //                     LED.solid(
    //                         Section.FULL,
    //                         DriverStation.getAlliance().get() == Alliance.Red
    //                             ? Color.kRed
    //                             : Color.kBlue))));

    // LED.registerSignal(
    //     6,
    //     controller.rightBumper(),
    //     buf -> {
    //       // iterate through buffer and apply colors as desired
    //     });
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
