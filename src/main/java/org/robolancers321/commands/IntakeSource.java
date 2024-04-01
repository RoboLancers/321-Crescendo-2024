/* (C) Robolancers 2024 */
package org.robolancers321.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.robolancers321.subsystems.launcher.Flywheel;
import org.robolancers321.subsystems.launcher.Indexer;
import org.robolancers321.subsystems.launcher.Pivot;

public class IntakeSource extends SequentialCommandGroup {
  private Flywheel flywheel;
  // private Sucker sucker;
  private Indexer indexer;
  private Pivot pivot;

  public IntakeSource() {
    this.flywheel = Flywheel.getInstance();
    // this.sucker = Sucker.getInstance();
    this.indexer = Indexer.getInstance();
    this.pivot = Pivot.getInstance();

    this.addCommands(
        new ParallelDeadlineGroup(this.indexer.intakeSource(), this.flywheel.intakeSource()),
        new Shift());
  }
}
