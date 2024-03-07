/* (C) Robolancers 2024 */
package org.robolancers321.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.robolancers321.subsystems.intake.Sucker;

public class PathAndShoot extends SequentialCommandGroup {
  private Sucker sucker;

  public PathAndShoot(String pathName) {
    this.sucker = Sucker.getInstance();

    this.addCommands(
        new ParallelRaceGroup(
            AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory(pathName)),
            new IntakeNote()),
        new ScoreSpeakerFromDistance().onlyIf(this.sucker::noteDetected));
  }
}
