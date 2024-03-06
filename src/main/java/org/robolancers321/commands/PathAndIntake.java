/* (C) Robolancers 2024 */
package org.robolancers321.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PathAndIntake extends SequentialCommandGroup {

  public PathAndIntake(String pathName) {

    this.addCommands(
        new ParallelCommandGroup(
            AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory(pathName)),
            new IntakeNote()));
  }
}
