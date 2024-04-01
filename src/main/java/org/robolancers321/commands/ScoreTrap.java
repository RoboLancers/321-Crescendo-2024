/* (C) Robolancers 2024 */
package org.robolancers321.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.robolancers321.subsystems.intake.Retractor;
import org.robolancers321.subsystems.launcher.Flywheel;
import org.robolancers321.subsystems.launcher.Indexer;
import org.robolancers321.subsystems.launcher.Pivot;

public class ScoreTrap extends SequentialCommandGroup {
  private Indexer indexer;
  private Flywheel flywheel;
  private Pivot pivot;
  private Retractor retractor;

  public ScoreTrap() {
    this.indexer = Indexer.getInstance();
    this.flywheel = Flywheel.getInstance();
    this.pivot = Pivot.getInstance();
    this.retractor = Retractor.getInstance();

    // may have to move retractor to balance

    this.addCommands(
        new ParallelDeadlineGroup(
            this.indexer.shiftFromHandoffForward(), this.flywheel.shiftForward()),
        this.indexer.revTrap(),
        this.flywheel.shiftBackwardFast());
  }
}
