/* (C) Robolancers 2024 */
package org.robolancers321.commands.AutoCommands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.robolancers321.subsystems.intake.Retractor;

public class PathAndRetract extends SequentialCommandGroup {
  private Retractor retractor;

  public PathAndRetract(PathPlannerPath path) {
    this.retractor = Retractor.getInstance();

    this.addCommands(
        new ParallelCommandGroup(AutoBuilder.followPath(path), this.retractor.moveToRetracted()));
  }
}
