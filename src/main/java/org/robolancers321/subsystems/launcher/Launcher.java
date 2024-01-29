package org.robolancers321.subsystems.launcher;

import org.robolancers321.subsystems.launcher.AimTable.AimCharacteristic;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class Launcher extends SubsystemBase {
    /*
     * Singleton
     */

    private static Launcher instance = null;

    public static Launcher getInstance(){
        if (instance == null) instance = new Launcher();

        return instance;
    }

    /*
     * Constants
     */

    // TODO: beam break(s)

    /*
     * Implementation
     */

    public Pivot pivot;
    public Indexer indexer;
    public Flywheel flywheel;

    // TODO: beam break(s)

    private Launcher(){
        this.pivot = Pivot.getInstance();
        this.indexer = Indexer.getInstance();
        this.flywheel = Flywheel.getInstance();
    }
    
    public Command yeetAmp(){
        return new SequentialCommandGroup(
            pivot.aimAtAmp(),
            new ParallelCommandGroup(
                flywheel.yeetNoteAmp(),
                indexer.outtake(() -> true), // TODO: pass in beam break state
                new WaitCommand(0.2)
            )
        );
    }

    // TODO: how to go about ths for active tracking
    public Command yeetSpeaker(double distance){
        AimCharacteristic aimCharacteristic = AimTable.getSpeakerAimCharacteristic(distance);

        return new SequentialCommandGroup(
            indexer.reindex(() -> true), // TODO: pass in beam break state
            new ParallelCommandGroup(
                flywheel.yeetNoteSpeaker(() -> aimCharacteristic.rpm),
                new SequentialCommandGroup(
                    pivot.aimAtSpeaker(() -> aimCharacteristic.angle),
                    new WaitUntilCommand(flywheel::isRevved), // TODO: maybe put a timeout here to be safe
                    indexer.outtake(() -> true), // TODO: pass in beam break state
                    new WaitCommand(0.2)
                )
            )
        );
    }
}
