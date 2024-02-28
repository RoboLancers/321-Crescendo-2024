package org.robolancers321.commands;

import org.robolancers321.subsystems.intake.Retractor;
import org.robolancers321.subsystems.intake.Sucker;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class DeployIntake extends ParallelCommandGroup {
    private Retractor retractor;
    private Sucker sucker;

    public DeployIntake(){
        this.retractor = Retractor.getInstance();
        this.sucker = Sucker.getInstance();

        this.addCommands(
            this.retractor.moveToIntake(),
            this.sucker.in()
        );
    }
}
