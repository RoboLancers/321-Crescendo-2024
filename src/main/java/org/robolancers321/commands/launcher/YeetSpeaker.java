/* (C) Robolancers 2024 */
package org.robolancers321.commands.launcher;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.function.DoubleSupplier;
import org.robolancers321.subsystems.launcher.Indexer;
import org.robolancers321.subsystems.launcher.Pivot;
import org.robolancers321.subsystems.launcher.Shooter;

public class YeetSpeaker extends ParallelCommandGroup {

  private Indexer indexer = Indexer.getInstance();

  private Pivot pivot = Pivot.getInstance();

  private Shooter shooter = Shooter.getInstance();

  public YeetSpeaker(DoubleSupplier distanceFromSpeaker) {

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
