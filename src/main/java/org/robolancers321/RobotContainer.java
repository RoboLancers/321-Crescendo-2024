/* (C) Robolancers 2024 */
package org.robolancers321;

import org.robolancers321.subsystem.Sucker;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {
  private Sucker sucker;

  public RobotContainer() {
    this.sucker = Sucker.getInstance();

    this.sucker.setDefaultCommand(this.sucker.tuneController());

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
