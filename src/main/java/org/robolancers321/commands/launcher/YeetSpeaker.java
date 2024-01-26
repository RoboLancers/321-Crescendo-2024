/* (C) Robolancers 2024 */
package org.robolancers321.commands.launcher;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.function.DoubleSupplier;
import org.robolancers321.subsystems.launcher.Indexer;
import org.robolancers321.subsystems.launcher.Pivot;
import org.robolancers321.subsystems.launcher.Shooter;

public class YeetSpeaker extends ParallelCommandGroup {
  private Indexer indexer;
  private Pivot pivot;
  private Shooter shooter;

  public YeetSpeaker(DoubleSupplier distanceFromSpeaker) {
    this.indexer = Indexer.getInstance();
    this.pivot = Pivot.getInstance();
    this.shooter = Shooter.getInstance();

    addRequirements(indexer, pivot, shooter);
    // Run shooter and aim at the same time, index only after at setpoint
    addCommands(
        shooter.yeetNoteSpeaker(distanceFromSpeaker),
        new SequentialCommandGroup(
            pivot.aimAtSpeaker(distanceFromSpeaker),
            indexer.index() // TODO: only run this when shooter is at speed
            ));
  }
}
