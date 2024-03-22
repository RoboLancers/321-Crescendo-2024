/* (C) Robolancers 2024 */
package org.robolancers321.commands.AutoCommands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.robolancers321.commands.Mate;
import org.robolancers321.commands.Shift;
import org.robolancers321.subsystems.intake.Retractor;
import org.robolancers321.subsystems.intake.Sucker;

public class PathAndMate extends SequentialCommandGroup {
  private Retractor retractor;
  private Sucker sucker;

  public PathAndMate(PathPlannerPath path) {
    this.retractor = Retractor.getInstance();
    this.sucker = Sucker.getInstance();

    this.addCommands(
        new ParallelCommandGroup(
            AutoBuilder.followPath(path),
            (new Mate().andThen(new Shift())).onlyIf(this.sucker::noteDetected)));
  }
}
