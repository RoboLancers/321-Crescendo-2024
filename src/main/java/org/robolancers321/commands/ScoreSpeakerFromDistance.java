/* (C) Robolancers 2024 */
package org.robolancers321.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.robolancers321.subsystems.drivetrain.Drivetrain;
import org.robolancers321.subsystems.intake.Retractor;
import org.robolancers321.subsystems.launcher.AimTable;
import org.robolancers321.subsystems.launcher.Flywheel;
import org.robolancers321.subsystems.launcher.Indexer;
import org.robolancers321.subsystems.launcher.Pivot;

public class ScoreSpeakerFromDistance extends SequentialCommandGroup {
  private Pivot pivot;
  private Retractor retractor;
  private Indexer indexer;
  private Flywheel flywheel;
  private Drivetrain drivetrain;

  public ScoreSpeakerFromDistance() {
    this.pivot = Pivot.getInstance();
    this.retractor = Retractor.getInstance();
    this.indexer = Indexer.getInstance();
    this.flywheel = Flywheel.getInstance();
    this.drivetrain = Drivetrain.getInstance();

    this.addCommands(
        new ParallelCommandGroup(
            new SequentialCommandGroup(this.drivetrain.turnToSpeaker(), this.drivetrain.stop()),
            new SequentialCommandGroup(
                new Mate(), this.retractor.moveToClearFromLauncher(), new Shift())),
        new ParallelCommandGroup(
            this.pivot.aimAtSpeaker(
                () -> AimTable.interpolatePivotAngle(this.drivetrain.getDistanceToSpeaker())),
            this.flywheel.revSpeakerFromRPM(
                () -> AimTable.interpolateFlywheelRPM(this.drivetrain.getDistanceToSpeaker()))),
        this.indexer.outtake(),
        this.indexer.off(),
        this.flywheel.off());
  }
}
