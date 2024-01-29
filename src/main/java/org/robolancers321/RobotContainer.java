/* (C) Robolancers 2024 */
package org.robolancers321;

import org.robolancers321.subsystems.launcher.Launcher;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {
  Launcher launcher;

  public RobotContainer() {
    this.launcher = Launcher.getInstance();

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
