/* (C) Robolancers 2024 */
package org.robolancers321.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.robolancers321.subsystems.launcher.Flywheel;
import org.robolancers321.subsystems.launcher.Indexer;
import org.robolancers321.subsystems.launcher.Pivot;

public class ScoreSpeakerFixedTeleop extends ParallelCommandGroup {
  private Pivot pivot;
  private Indexer indexer;
  private Flywheel flywheel;

  // assumes mating has finished
  public ScoreSpeakerFixedTeleop() {
    this.pivot = Pivot.getInstance();
    this.indexer = Indexer.getInstance();
    this.flywheel = Flywheel.getInstance();
    
    this.addCommands(
      new Shift().andThen(flywheel.revSpeaker()),
      pivot.moveToRetracted(),
      Commands.idle()
    );
  }
}
