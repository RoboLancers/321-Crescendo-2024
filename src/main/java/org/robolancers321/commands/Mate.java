/* (C) Robolancers 2024 */
package org.robolancers321.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.robolancers321.subsystems.intake.Retractor;
import org.robolancers321.subsystems.intake.Sucker;
import org.robolancers321.subsystems.launcher.Flywheel;
import org.robolancers321.subsystems.launcher.Indexer;
import org.robolancers321.subsystems.launcher.Pivot;

public class Mate extends SequentialCommandGroup {
  private Retractor retractor;
  private Sucker sucker;
  private Pivot pivot;
  private Indexer indexer;
  private Flywheel flywheel;

  public Mate() {
    this.retractor = Retractor.getInstance();
    this.sucker = Sucker.getInstance();
    this.pivot = Pivot.getInstance();
    this.indexer = Indexer.getInstance();
    this.flywheel = Flywheel.getInstance();

    this.addCommands(
        this.flywheel.off(),
        new ParallelCommandGroup(this.retractor.moveToMating(), this.pivot.moveToMating()),
        this.indexer
            .acceptHandoff()
            .alongWith(this.sucker.out(), this.flywheel.acceptHandoff())
            .until(this.indexer::exitBeamBroken),
        // new ParallelDeadlineGroup(
        //     this.indexer.acceptHandoff(), this.sucker.out(), this.flywheel.acceptHandoff()),
        // new ParallelDeadlineGroup(
        //     this.indexer.shiftFromHandoffForward(),
        //     this.sucker.out(),
        //     this.flywheel.shiftForward()),
        this.sucker.off(),
        this.indexer.off(),
        this.flywheel.off(),
        new ParallelCommandGroup(this.retractor.moveToRetracted()));
    // this.pivot.moveToRetracted()));

    this.setName("Mate");
  }
}
