/* (C) Robolancers 2024 */
package org.robolancers321;

import org.robolancers321.subsystems.intake.Intake;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  Intake intake;

  XboxController controller;

  public RobotContainer() {
    this.intake = Intake.getInstance();
    this.controller = new XboxController(1);

    this.configureBindings();
  }

  private void configureBindings() {
    new Trigger(this.controller::getAButton).whileTrue(this.intake.sucker.tuneController());
    new Trigger(this.controller::getBButton).whileTrue(this.intake.retractor.tuneControllers());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
