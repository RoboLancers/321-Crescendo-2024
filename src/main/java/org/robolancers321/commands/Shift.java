/* (C) Robolancers 2024 */
package org.robolancers321.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.robolancers321.subsystems.launcher.Flywheel;
import org.robolancers321.subsystems.launcher.Indexer;

public class Shift extends SequentialCommandGroup {
  private Indexer indexer;
  private Flywheel flywheel;

  public Shift() {
    this.indexer = Indexer.getInstance();
    this.flywheel = Flywheel.getInstance();

    this.addCommands(
        this.indexer.shiftForward(),
        this.indexer.shiftBackward().raceWith(this.flywheel.revSpeakerFromRPM(() -> -300)));
  }
}
