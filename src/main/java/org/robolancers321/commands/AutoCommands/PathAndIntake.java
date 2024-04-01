/* (C) Robolancers 2024 */
package org.robolancers321.commands.AutoCommands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.robolancers321.commands.IntakeNoteManual;
import org.robolancers321.subsystems.intake.Retractor;
import org.robolancers321.subsystems.intake.Sucker;

public class PathAndIntake extends SequentialCommandGroup {

  Retractor retractor;
  Sucker sucker;

  public PathAndIntake(PathPlannerPath path) {

    this.retractor = Retractor.getInstance();
    this.sucker = Sucker.getInstance();

    this.addCommands(
        retractor.moveToIntake().withTimeout(0.5),
        new ParallelRaceGroup(AutoBuilder.followPath(path), new IntakeNoteManual()));
  }
}
