/* (C) Robolancers 2024 */
package org.robolancers321.commands.launcher;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.robolancers321.subsystems.launcher.Indexer;
import org.robolancers321.subsystems.launcher.Pivot;
import org.robolancers321.subsystems.launcher.Shooter;

public class YeetAmp extends SequentialCommandGroup {

  private Indexer indexer;
  private Pivot pivot;
  private Shooter shooter;

  public YeetAmp() {
    this.indexer = Indexer.getInstance();
    this.pivot = Pivot.getInstance();
    this.shooter = Shooter.getInstance();

    addRequirements(indexer, pivot, shooter);
    addCommands(
        pivot.positionAmp(), new ParallelCommandGroup(shooter.yeetNoteAmp(), indexer.index()));
  }
}
