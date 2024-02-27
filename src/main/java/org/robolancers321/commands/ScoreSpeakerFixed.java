/* (C) Robolancers 2024 */
package org.robolancers321.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.robolancers321.subsystems.intake.Intake;
import org.robolancers321.subsystems.launcher.Launcher;

public class ScoreSpeakerFixed extends SequentialCommandGroup {
  private Intake intake;
  private Launcher launcher;

  public ScoreSpeakerFixed() {
    this.intake = Intake.getInstance();
    this.launcher = Launcher.getInstance();

    this.addCommands(
        this.launcher.pivot.aimAtSpeakerFixed(),
        this.launcher.flywheel.yeetNoteSpeakerFixed(),
        new WaitCommand(1.5),
        new ParallelRaceGroup(
            this.launcher.indexer.outtake(() -> false),
            this.intake.sucker.out(),
            new WaitCommand(0.8)),
        this.launcher.flywheel.off(),
        this.launcher.pivot.moveToRetracted());
  }
}
