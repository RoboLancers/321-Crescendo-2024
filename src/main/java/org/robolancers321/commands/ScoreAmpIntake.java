/* (C) Robolancers 2024 */
package org.robolancers321.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.robolancers321.subsystems.intake.Retractor;
import org.robolancers321.subsystems.intake.Sucker;

public class ScoreAmpIntake extends ParallelRaceGroup {
  private Retractor retractor;
  private Sucker sucker;

  public ScoreAmpIntake() {
    this.retractor = Retractor.getInstance();
    this.sucker = Sucker.getInstance();

    this.addCommands(
        new WaitCommand(3.0),
        this.retractor
            .moveToAmp()
            .until(() -> this.retractor.atGoalTimed(0.5))
            .andThen(this.sucker.ampShot().withTimeout(0.4))

        //  new ParallelDeadlineGroup(
        //     new WaitCommand(0.4), this.sucker.ampShot(), Commands.idle(this.retractor))
        );
  }
}
