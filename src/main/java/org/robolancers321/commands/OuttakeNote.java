package org.robolancers321.commands;

import org.robolancers321.subsystems.intake.Retractor;
import org.robolancers321.subsystems.intake.Sucker;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class OuttakeNote extends SequentialCommandGroup {
    private Retractor retractor;
    private Sucker sucker;

    public OuttakeNote(){
        this.retractor = Retractor.getInstance();
        this.sucker = Sucker.getInstance();
        
        this.addCommands(
            this.retractor.moveToOuttake(),
            this.sucker.out()
        );
    }
}
