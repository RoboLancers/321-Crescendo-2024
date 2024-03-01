/* (C) Robolancers 2024 */
package org.robolancers321.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.robolancers321.subsystems.intake.Retractor;
import org.robolancers321.subsystems.launcher.Flywheel;
import org.robolancers321.subsystems.launcher.Pivot;

public class ScoreSpeakerFixed extends SequentialCommandGroup {
  private Retractor retractor;
  private Pivot pivot;
  private Flywheel flywheel;

  public ScoreSpeakerFixed() {
    this.retractor = Retractor.getInstance();
    this.pivot = Pivot.getInstance();
    this.flywheel = Flywheel.getInstance();

    this.addCommands(
        new ParallelCommandGroup(this.retractor.moveToMating(), this.pivot.moveToMating()),
        this.flywheel.revSpeaker(),
        new RunCommand(() -> {}));
  }
}
