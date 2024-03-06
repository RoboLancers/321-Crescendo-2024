package org.robolancers321.commands;

import org.robolancers321.subsystems.drivetrain.Drivetrain;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoPickupNote extends ParallelRaceGroup {
    private Drivetrain drivetrain;
    
    public AutoPickupNote(){
        this.drivetrain = Drivetrain.getInstance();   

        this.addCommands(
            new IntakeNote(),
            new SequentialCommandGroup(
                this.drivetrain.turnToNote(),
                new SequentialCommandGroup(
                    this.drivetrain.driveCommand(() -> 0.1, () -> 0.0, () -> 0.0, () -> false).onlyWhile(this.drivetrain::seesNote),
                    this.drivetrain.driveCommand(() -> 0.1, () -> 0.0, () -> 0.0, () -> false).withTimeout(0.5)
                ).onlyIf(this.drivetrain::seesNote)
            )
        );
    }
}
