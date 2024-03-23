/* (C) Robolancers 2024 */
package org.robolancers321.commands.AutoCommands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.robolancers321.commands.IntakeNote;
import org.robolancers321.commands.Mate;
import org.robolancers321.commands.ScoreSpeakerFromDistance;
import org.robolancers321.commands.Shift;
import org.robolancers321.subsystems.intake.Retractor;
import org.robolancers321.subsystems.intake.Sucker;

public class PathAndShoot extends SequentialCommandGroup {
  private Sucker sucker;
  private Retractor retractor;

  public PathAndShoot(PathPlannerPath path) {
    this.sucker = Sucker.getInstance();
    this.retractor = Retractor.getInstance();

    this.addCommands(
        new ParallelRaceGroup(AutoBuilder.followPath(path), new IntakeNote()),
        (new Mate().andThen(new Shift()).andThen(new ScoreSpeakerFromDistance()))
            .onlyIf(this.sucker::noteDetected));
  }
}
