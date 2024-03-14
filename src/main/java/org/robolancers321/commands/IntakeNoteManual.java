/* (C) Robolancers 2024 */
package org.robolancers321.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.robolancers321.subsystems.intake.Retractor;
import org.robolancers321.subsystems.intake.Sucker;
import org.robolancers321.subsystems.launcher.Pivot;

public class IntakeNoteManual extends ParallelCommandGroup {
    private Retractor retractor;
    private Sucker sucker;

    public IntakeNoteManual() {
        this.retractor = Retractor.getInstance();
        this.sucker = Sucker.getInstance();

        this.addCommands(
          this.retractor.moveToIntake(),
          this.sucker.in()
        );
    }
}
