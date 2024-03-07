/* (C) Robolancers 2024 */
package org.robolancers321.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.robolancers321.subsystems.intake.Retractor;
import org.robolancers321.subsystems.intake.Sucker;

public class IntakeNote extends SequentialCommandGroup {
  private Retractor retractor;
  private Sucker sucker;

  public IntakeNote() {
    this.retractor = Retractor.getInstance();
    this.sucker = Sucker.getInstance();

    this.addCommands(
        new ParallelDeadlineGroup(
            new WaitUntilCommand(this.sucker::noteDetected),
            this.retractor.moveToIntake(),
            this.sucker.in()),
        this.sucker.offInstantly(),
        this.retractor.moveToMating());
  }
}
