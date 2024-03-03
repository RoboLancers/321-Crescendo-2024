/* (C) Robolancers 2024 */
package org.robolancers321.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.robolancers321.subsystems.intake.Retractor;
import org.robolancers321.subsystems.intake.Sucker;

public class IntakeNote extends ParallelCommandGroup {
  private Retractor retractor;
  private Sucker sucker;

  public IntakeNote() {
    this.retractor = Retractor.getInstance();
    this.sucker = Sucker.getInstance();

    this.addCommands(retractor.moveToIntake(), sucker.in());
  }
}
