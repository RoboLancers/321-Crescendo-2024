/* (C) Robolancers 2024 */
package org.robolancers321.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.robolancers321.subsystems.launcher.Flywheel;
import org.robolancers321.subsystems.launcher.Indexer;
import org.robolancers321.subsystems.launcher.Pivot;

public class ScoreAmp extends SequentialCommandGroup {
  private Pivot pivot;
  private Indexer indexer;
  private Flywheel flywheel;

  public ScoreAmp() {
    this.pivot = Pivot.getInstance();
    this.indexer = Indexer.getInstance();
    this.flywheel = Flywheel.getInstance();

    this.addCommands(
        new Shift(),
        new ParallelCommandGroup(this.pivot.aimAtAmp(), this.flywheel.revAmp()),
        this.indexer.outtake(),
        this.flywheel.off(),
        this.pivot.moveToRetracted());
  }
}
