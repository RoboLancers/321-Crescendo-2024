package org.robolancers321.commands;

import org.robolancers321.subsystems.intake.Intake;
import org.robolancers321.subsystems.launcher.Launcher;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ScoreSpeakerFixed extends SequentialCommandGroup {
    private Intake intake;
    private Launcher launcher;

    public ScoreSpeakerFixed(){
        this.intake = Intake.getInstance();
        this.launcher = Launcher.getInstance();

        this.addCommands(
            this.launcher.pivot.aimAtSpeakerFixed(),
            new ParallelRaceGroup(
                this.launcher.flywheel.yeetNoteSpeakerFixed(),
                new SequentialCommandGroup(
                    new WaitCommand(1.5),
                    new ParallelRaceGroup(
                        this.launcher.indexer.outtake(() -> false),
                        this.intake.sucker.out(),
                        new WaitCommand(0.8)
                    )
                )
            ),
            this.launcher.pivot.moveToRetracted()
        );
    }
}
