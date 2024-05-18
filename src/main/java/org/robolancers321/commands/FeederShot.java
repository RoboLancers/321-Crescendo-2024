/* (C) Robolancers 2024 */
package org.robolancers321.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.robolancers321.subsystems.intake.Retractor;
import org.robolancers321.subsystems.launcher.Flywheel;
import org.robolancers321.subsystems.launcher.Indexer;
import org.robolancers321.subsystems.launcher.Pivot;

public class FeederShot extends ParallelCommandGroup {
  private Pivot pivot;
  private Retractor retractor;
  private Indexer indexer;
  private Flywheel flywheel;

  // assumes mating has finished
  public FeederShot() {
    this.pivot = Pivot.getInstance();
    this.retractor = Retractor.getInstance();
    this.indexer = Indexer.getInstance();
    this.flywheel = Flywheel.getInstance();

    this.addCommands(
        flywheel.revFeeder(), retractor.moveToMating(), pivot.aimAtSpeakerFixed(), Commands.idle());
  }
}
