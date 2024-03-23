/* (C) Robolancers 2024 */
package org.robolancers321.commands.AutoCommands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.robolancers321.commands.AutoPickupNote;
import org.robolancers321.subsystems.intake.Retractor;
import org.robolancers321.subsystems.intake.Sucker;

public class PathAndAutoPickup extends SequentialCommandGroup {

  Retractor retractor;
  Sucker sucker;

  public PathAndAutoPickup(PathPlannerPath path) {

    this.retractor = Retractor.getInstance();
    this.sucker = Sucker.getInstance();

    this.addCommands(AutoBuilder.followPath(path), new AutoPickupNote());
  }
}
