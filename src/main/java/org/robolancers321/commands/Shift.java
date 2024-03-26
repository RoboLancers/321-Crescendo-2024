/* (C) Robolancers 2024 */
package org.robolancers321.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.robolancers321.subsystems.launcher.Flywheel;
import org.robolancers321.subsystems.launcher.Indexer;
import org.robolancers321.subsystems.launcher.Pivot;

public class Shift extends SequentialCommandGroup {
  private Indexer indexer;
  private Flywheel flywheel;
  private Pivot pivot;

  public Shift() {
    this.indexer = Indexer.getInstance();
    this.flywheel = Flywheel.getInstance();
    this.pivot = Pivot.getInstance();

    this.addCommands(
        this.pivot.moveToShift(),
        this.flywheel.shiftBackwardFast(),
        this.indexer.shiftBackFromExit(),
        this.flywheel.shiftBackwardSlow(),
        this.indexer.shiftBackToEntrance(),
        this.indexer.shiftForwardToEntrance(),
        this.indexer.off(),
        this.flywheel.off());
  }
}
