/* (C) Robolancers 2024 */
package org.robolancers321;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.robolancers321.subsystems.intake.Retractor;

public class RobotContainer {
  Retractor retractor;

  public RobotContainer() {
    this.retractor = Retractor.getInstance();

    this.retractor.setDefaultCommand(this.retractor.tuneControllers());

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
