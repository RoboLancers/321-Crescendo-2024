/* (C) Robolancers 2024 */
package org.robolancers321.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.robolancers321.subsystems.intake.Retractor;
import org.robolancers321.subsystems.intake.Sucker;
import org.robolancers321.subsystems.launcher.Flywheel;
import org.robolancers321.subsystems.launcher.Indexer;
import org.robolancers321.subsystems.launcher.Pivot;

public class ScoreSpeakerFixedAuto extends SequentialCommandGroup {
  private Retractor retractor;
  private Indexer indexer;
  private Pivot pivot;
  private Flywheel flywheel;
  private Sucker sucker;

  // assumes arms are at mating with note inside sucker
  public ScoreSpeakerFixedAuto() {
    this.retractor = Retractor.getInstance();
    this.flywheel = Flywheel.getInstance();
    this.pivot = Pivot.getInstance();
    this.indexer = Indexer.getInstance();
    this.sucker = Sucker.getInstance();

    this.addCommands(
        new ParallelCommandGroup(
            this.flywheel.revSpeaker(),
            this.retractor.moveToSpeaker(),
            this.pivot.aimAtSpeakerFixed()),
        new ParallelDeadlineGroup(this.indexer.acceptHandoff(), this.sucker.out())
        // this.indexer.off() // ,
        // stay revved during auto
        // this.flywheel.off()
        );
  }
}
