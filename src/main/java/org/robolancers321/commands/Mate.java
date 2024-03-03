/* (C) Robolancers 2024 */
package org.robolancers321.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.robolancers321.subsystems.intake.Retractor;
import org.robolancers321.subsystems.intake.Sucker;
import org.robolancers321.subsystems.launcher.Indexer;
import org.robolancers321.subsystems.launcher.Pivot;

public class Mate extends SequentialCommandGroup {
  private Retractor retractor;
  private Sucker sucker;
  private Pivot pivot;
  private Indexer indexer;

  public Mate() {
    this.retractor = Retractor.getInstance();
    this.sucker = Sucker.getInstance();
    this.pivot = Pivot.getInstance();
    this.indexer = Indexer.getInstance();

    this.addCommands(
        new ParallelCommandGroup(this.retractor.moveToMating(), this.pivot.moveToMating()),
        new ParallelRaceGroup(this.sucker.out(), this.indexer.acceptHandoff()),
        this.retractor.moveToClearFromLauncher());
  }
}
