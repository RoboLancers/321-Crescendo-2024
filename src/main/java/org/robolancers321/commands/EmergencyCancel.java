/* (C) Robolancers 2024 */
package org.robolancers321.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.robolancers321.subsystems.drivetrain.Drivetrain;
import org.robolancers321.subsystems.intake.Retractor;
import org.robolancers321.subsystems.intake.Sucker;
import org.robolancers321.subsystems.launcher.Flywheel;
import org.robolancers321.subsystems.launcher.Indexer;
import org.robolancers321.subsystems.launcher.Pivot;

public class EmergencyCancel extends InstantCommand {
  public EmergencyCancel() {
    super(
        () -> {},
        Drivetrain.getInstance(),
        Retractor.getInstance(),
        Sucker.getInstance(),
        Indexer.getInstance(),
        Flywheel.getInstance(),
        Pivot.getInstance());
  }
}
