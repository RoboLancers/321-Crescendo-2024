/* (C) Robolancers 2024 */
package org.robolancers321.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
        this.indexer
            .shiftBackFromExit()
            .alongWith(this.flywheel.shiftBackwardFast())
            .until(this.indexer::exitBeamNotBroken),
        this.indexer
            .shiftForwardToEntrance()
            .alongWith(this.flywheel.shiftBackwardSlow())
            .until(this.indexer::entranceBeamBroken),
        new WaitCommand(0.2),
        this.indexer.off(),
        new WaitCommand(0.2),
        this.flywheel.off());

    this.setName("Shift");
  }
}
