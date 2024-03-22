package org.robolancers321.commands.AutoCommands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.robolancers321.subsystems.intake.Retractor;
import org.robolancers321.subsystems.intake.Sucker;

public class PathAndRetract extends SequentialCommandGroup {
  private Retractor retractor;

  public PathAndRetract(PathPlannerPath path) {
    this.retractor = Retractor.getInstance();

    this.addCommands(
        new ParallelCommandGroup(
            AutoBuilder.followPath(path),
            this.retractor.moveToRetracted()
        ));
  }
}