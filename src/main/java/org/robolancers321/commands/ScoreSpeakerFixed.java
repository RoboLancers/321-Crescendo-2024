/* (C) Robolancers 2024 */
package org.robolancers321.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.robolancers321.subsystems.intake.Sucker;
import org.robolancers321.subsystems.launcher.Flywheel;
import org.robolancers321.subsystems.launcher.Indexer;
import org.robolancers321.subsystems.launcher.Pivot;

public class ScoreSpeakerFixed {
  // assumes mating has finished
  public static Command rev() {
    Pivot pivot = Pivot.getInstance();
    Flywheel flywheel = Flywheel.getInstance();

    return new SequentialCommandGroup(
        pivot.moveToRetracted(), flywheel.revSpeaker(), new RunCommand(() -> {}));
  }

  public static Command eject() {
    Indexer indexer = Indexer.getInstance();
    Sucker sucker = Sucker.getInstance();
    Flywheel flywheel = Flywheel.getInstance();

    return new SequentialCommandGroup(
        new ParallelRaceGroup(indexer.acceptHandoff().andThen(indexer.outtake()), sucker.out()),
        flywheel.off());
  }
}
