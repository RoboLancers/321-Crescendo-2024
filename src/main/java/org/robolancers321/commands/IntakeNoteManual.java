/* (C) Robolancers 2024 */
package org.robolancers321.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.robolancers321.subsystems.intake.Retractor;
import org.robolancers321.subsystems.intake.Sucker;

public class IntakeNoteManual extends SequentialCommandGroup {
  private Retractor retractor;
  private Sucker sucker;

  public IntakeNoteManual() {
    this.retractor = Retractor.getInstance();
    this.sucker = Sucker.getInstance();

    this.addCommands(new ParallelCommandGroup(this.retractor.moveToIntake(), this.sucker.in()));
  }
}
