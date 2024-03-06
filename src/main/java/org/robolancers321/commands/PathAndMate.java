/* (C) Robolancers 2024 */
package org.robolancers321.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.robolancers321.subsystems.intake.Retractor;
import org.robolancers321.subsystems.intake.Sucker;

public class PathAndMate extends SequentialCommandGroup {
  private Sucker sucker;
  private Retractor retractor;

  public PathAndMate(String pathName) {
    this.sucker = Sucker.getInstance();
    this.retractor = Retractor.getInstance();

    this.addCommands(
        new ParallelCommandGroup(
            AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory(pathName)),
            new ConditionalCommand(
                new Mate(),
                new InstantCommand(
                    () -> {
                      retractor.moveToMating();
                    }),
                this.sucker::noteDetected)));
  }
}
