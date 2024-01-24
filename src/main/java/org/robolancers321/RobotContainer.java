/* (C) Robolancers 2024 */
package org.robolancers321;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.robolancers321.subsystem.Intake;

public class RobotContainer {

  public Intake intake = new Intake();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
